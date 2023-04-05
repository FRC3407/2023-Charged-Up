#include <iostream>
#include <csignal>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <vector>
#include <array>
#include <span>
#include <string.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <fmt/format.h>

#include <cscore_cv.h>
#include <cscore_cpp.h>
#include <wpi/json.h>
#include <wpi/raw_ostream.h>
#include <wpi/raw_istream.h>
#include <wpi/StringExtras.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Pose3d.h>
#include <frc/apriltag/AprilTagFields.h>
#include <networktables/NetworkTable.h>
#include <networktables/IntegerTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <cameraserver/CameraServer.h>

#include <core/calib.h>
#include <core/neon.h>

#include <cpp-tools/src/sighandle.h>
#include <cpp-tools/src/unix/stats2.h>
#include <cpp-tools/src/unix/stats2.cpp>

#include "field.h"


#define DLOG(x) std::cout << (x) << std::endl;
#define DEBUG 2

#define FRC_CONFIG "/boot/frc.json"
#define NT_IDENTITY "Vision RPI"
#define SIM_ADDR "192.168.0.8"
#define NT_DEFAULT 0

using namespace std::chrono_literals;
using namespace std::chrono;

enum CamID {
	FWD_CAMERA,
	ARM_CAMERA,
	TOP_CAMERA,

	NUM_CAMERAS
};
static const nt::PubSubOptions
	NT_OPTIONS = { .periodic = 1.0 / 30.0 };
static const std::array<const char*, (size_t)CamID::NUM_CAMERAS>
	CAMERA_TAGS{ "forward", "arm", "top" };
static const cs::VideoMode
	DEFAULT_VMODE{ cs::VideoMode::kMJPEG, 640, 480, 30 };
static const int
	DEFAULT_EXPOSURE = 40,
	DEFAULT_WBALANCE = -1,
	DEFAULT_BRIGHTNESS = 50,
	DEFAULT_DOWNSCALE = 4;
static const CalibList
	STATIC_CALIBRATIONS{
		{
			{ "Microsoft® LifeCam HD-3000", {
				{ cv::Size{640, 480}, {
					cv::Mat1f{ {3, 3}, {
						673.6653136395231, 0, 339.861572657799,
						0, 666.1104961259615, 244.21065776461745,
						0, 0, 1
					} },
					cv::Mat1f{ {1, 5}, {
						0.04009256446529976, -0.4529245799337021,
						-0.001655316303789686, -0.00019284071985319236,
						0.5736326357832554
					} }
				} },
				{ cv::Size{1280, 720}, {
					cv::Mat1f{ {3, 3}, {
						1.0990191649538529e+03, 0, 6.2139182601803475e+02,
						0, 1.0952324445233039e+03, 3.5986298825678898e+02,
						0, 0, 1
					} },
					cv::Mat1f{ {1, 5}, {
						1.5547098669227585e-01, -1.3752756798809054e+00,
						9.5749479935394190e-04, 5.4056952639896377e-04,
						2.4270632150764744e+00
					} }
				} }
			} }
		}
	};
static const cv::Mat_<float>
	DEFAULT_CAM_MATX{cv::Mat_<float>::zeros(3, 3)},
	DEFAULT_CAM_DIST{cv::Mat_<float>::zeros(1, 5)};


class Stats : public wpi::Sendable, public CoreStats {
public:
	inline Stats(bool all_cores = false) : CoreStats(all_cores) {}

	virtual void InitSendable(wpi::SendableBuilder& b) override {
		b.AddFloatProperty("Core temp", [this](){ return this->temp(); }, nullptr);
		b.AddFloatProperty("Utilization", [this](){ return this->fromLast(); }, nullptr);
		b.AddFloatProperty("Avg FTime", [this](){ return (float)this->ftime_avg; }, nullptr);
	}

	std::atomic<float> ftime_avg{};

};


class VideoSinkImpl : public cs::VideoSink {
public:
	inline VideoSinkImpl(CS_Sink h) : VideoSink(h) {}	// this constructor is protected so we have to subclass to use it publicly
};
struct CThread {
	CThread() = default;
	CThread(CThread&& t) :
		camera_matrix(std::move(t.camera_matrix)),
		dist_matrix(std::move(t.dist_matrix)),
		vmode(std::move(t.vmode)),
		vid(t.vid),
		camera_h(t.camera_h),
		fout_h(t.fout_h),
		fin_h(t.fin_h),
		link_state(t.link_state.load()),
		proc(std::move(t.proc)) {}

	cv::Mat_<float>
		camera_matrix{ DEFAULT_CAM_MATX },
		dist_matrix{ DEFAULT_CAM_MATX };
	cs::VideoMode vmode;
	cv::Mat frame, buff;
	std::atomic<float> ftime{0.f};

	struct ProcBuff {
		// using PVec = std::array<float, 3>;
		using PVec = cv::Mat_<float>;

		std::vector<std::vector<cv::Point2f> > corners;
		std::vector<int32_t> ids;
		std::vector<PVec> tvecs, rvecs;
		cv::Mat fbuff;
	} *vproc_buffer{nullptr};

	int vid{-1};
	CS_Source camera_h, fout_h;
	CS_Sink fin_h, view_h;

	std::atomic<bool> link_state{true};
	std::atomic<int> procm{0};
	std::thread proc;

};
void _update(CThread&);	// these are like instance methods
void _worker(CThread&);
void _shutdown(CThread&);


bool init(const char* f = FRC_CONFIG);

struct {
	system_clock::time_point start_time;
	struct {
		std::atomic<bool>
			program_enable{true},
			view_updated{false},
			vrbo_updated{false},
			exposure_updated{false},
			dscale_updated{false}
		;
	} state;
	Stats stats{};

	std::shared_ptr<nt::NetworkTable> base_ntable;
	struct {
		nt::IntegerEntry
			views_avail,	// the amount of cameras connected
			view_id,		// active camera id
			ovl_verbosity,	// overlay verbosity - 0 for off, 1+ for increasing verbosity outputs
			april_mode,		// apriltag detection mode - -2 for off, -1 for active camera, 0+ for specific camera
			retro_mode,		// retrorefl tape detection mode - -2 for off, -1 for active camera, 0+ for specific camera
			exposure,		// exposure to apply to all cameras - default 40
			downscale		// outputs are downscaled by this factor
		;
		nt::DoubleArrayEntry
			poses;
	} nt;

	int next_stream_port = 1181;
	std::vector<CThread> cthreads;
	cv::Mat disconnect_frame;
	CS_Source discon_frame_h;
	CS_Sink stream_h;


	const cv::Ptr<cv::aruco::DetectorParameters> aprilp_params{ cv::aruco::DetectorParameters::create() };
	const cv::Ptr<cv::aruco::Board> aprilp_field{ ::FIELD_2023 };
	const frc::AprilTagFieldLayout aprilp_field_poses{ frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp) };
	std::vector<double> nt_pose_buffer{};

} _global;

// frc::Pose3d toPose3d(
// 	const CThread::ProcBuff::PVec&,
// 	const CThread::ProcBuff::PVec&);
// std::span<frc::Pose3d> toPose3d(
// 	std::vector<frc::Pose3d>&,
// 	const std::vector<CThread::ProcBuff::PVec>&,
// 	const std::vector<CThread::ProcBuff::PVec>&);
void _ap_detect_aruco(
	const cv::Mat&,
	std::vector<std::vector<cv::Point2f>>&,
	std::vector<int32_t>&);
int _ap_estimate_aruco(
	std::vector<std::vector<cv::Point2f>>& corners, std::vector<int32_t>& ids,
	cv::InputArray cmatx, cv::InputArray cdist,
	std::vector<frc::Pose3d>& estimations/*,
	std::vector<cv::Mat1f>& _tvecs, std::vector<cv::Mat1f>& _rvecs,
	std::vector<cv::Point3f>& _obj_points, std::vector<cv::Point2f>& _img_points*/);
// inline int _ap_estimate_aruco(
// 	std::vector<std::vector<cv::Point2f>>& corners, std::vector<int32_t>& ids,
// 	cv::InputArray cmatx, cv::InputArray cdist,
// 	std::vector<frc::Pose3d>& estimations,
// 	std::vector<cv::Mat1f>& _tvecs, std::vector<cv::Mat1f>& _rvecs
// ) {
// 	std::vector<cv::Point3f> _obj_points;
// 	std::vector<cv::Point2f> _img_points;
// 	_ap_estimate_aruco(corners, ids, cmatx, cdist, estimations, _tvecs, _rvecs, _obj_points, _img_points);
// }


void neonTest() {
	std::cout << "NEON Test - 'neon_deinterlace_wst()'\nInput array: [ ";
	uint8_t c3[48];
	uint8_t d[16];
	uint8_t d2[16];
	for(size_t i = 0; i < 48U; i++) {
		c3[i] = (i * 9 + 5) / 3;
		std::cout << (int)c3[i] << ", ";
	}
	std::cout << "]" << std::endl;
	for(size_t i = 0; i < 16U; i++) {
		int z = i * 3;
		d2[i] = c3[z + 0] - ((c3[z + 1]) + (c3[z + 2]) + 0);
	}
	memcpy_deinterlace_wst_asm(c3, d, 16U, 0U, 0xFF, 0xFF, 0x00);
	std::cout << "Output Array: [ ";
	for(size_t i = 0; i < 16U; i++) {
		int x = d[i];
		std::cout << x << ", ";
	}
	std::cout << "]\nShould Be: [ ";
	for(size_t i = 0; i < 16U; i++) {
		int x = d2[i];
		std::cout << x << ", ";
	}
	std::cout << "]\n\n" << std::endl;
}


int main(int argc, char** argv) {
	neonTest();
	int status = 0;
	// setup
	{
		_global.start_time = system_clock::now();
		init();

		struct sigaction _action;
		sigemptyset(&(_action.sa_mask));
		_action.sa_flags = SA_SIGINFO;
		_action.sa_sigaction = [&_global](int, siginfo_t*, void*){
			_global.state.program_enable = false;
		};
		sigaction((unsigned int)SigHandle::Sigs::INT, &_action, nullptr);
		sigaction((unsigned int)SigHandle::Sigs::QUIT, &_action, nullptr);
		sigaction((unsigned int)SigHandle::Sigs::ILL, &_action, nullptr);
		sigaction((unsigned int)SigHandle::Sigs::SEGV, &_action, nullptr);
	}

#if DEBUG > 0
	int i = 0;
	for(CThread& t : _global.cthreads) {
		fmt::print(
			"Camera Instance {}:\n\tVID: {}\n\tSource handle: {}\n"
			"\tCV in handle: {}\n\tCV out handle: {}\n\tVMODE:\n"
			"\t\tFrame Width: {}\n\t\tFrame Height: {}\n"
			"\t\tFPS: {}\n\t\tFormat: {}\n",
			i++, t.vid, t.camera_h, t.fin_h, t.fout_h,
			t.vmode.width, t.vmode.height, t.vmode.fps, t.vmode.pixelFormat
		);
	}
#endif

	// mainloop
	for(;_global.state.program_enable;) {

		_global.state.view_updated = (_global.nt.view_id.ReadQueue().size() > 0);
		_global.state.vrbo_updated = (_global.nt.ovl_verbosity.ReadQueue().size() > 0);
		_global.state.exposure_updated = (_global.nt.exposure.ReadQueue().size() > 0);
		_global.state.dscale_updated = (_global.nt.downscale.ReadQueue().size() > 0);

#if DEBUG > 1
		if(_global.state.view_updated) { std::cout << "MAINLOOP: View idx updated." << std::endl; }
		if(_global.state.vrbo_updated) { std::cout << "MAINLOOP: Verbosity lvl updated." << std::endl; }
		if(_global.state.exposure_updated) { std::cout << "MAINLOOP: Exposure updated." << std::endl; }
		if(_global.state.dscale_updated) { std::cout << "MAINLOOP: Downscale updated." << std::endl; }
#endif

		size_t i = 0;
		float a = 0;
		for(CThread& t : _global.cthreads) {
			_update(t);
			a += t.ftime;
			i++;
		}
		_global.stats.ftime_avg = a / i;

		frc::SmartDashboard::UpdateValues();

		std::this_thread::sleep_for(100ms);
	}

	// shutdown
	{
		std::cout << "\nShutting down threads..." << std::endl;
		for(CThread& t : _global.cthreads) {
			_shutdown(t);
		}
		cs::ReleaseSink(_global.stream_h, &status);
		cs::ReleaseSource(_global.discon_frame_h, &status);
		cs::Shutdown();
		std::cout << "Shutdown complete. Exitting..." << std::endl;
	}

	std::cout << "Runtime: " <<
		duration<double>(system_clock::now() -
			_global.start_time).count() << 's' << std::endl;

	return EXIT_SUCCESS;

}


bool loadJson(wpi::json& j, const char* file) {
	std::error_code ec;
    wpi::raw_fd_istream is(file, ec);
    if (ec) {
        wpi::errs() << "Could not open '" << file << "': " << ec.message() << newline;
        return false;
    }
    try { j = wpi::json::parse(is); }
    catch (const wpi::json::parse_error& e) {
        wpi::errs() << "Failed to parse JSON for " << file << /*": byte " << (int)e.byte <<*/ ": " << e.what() << newline;
        return false;
    }
    if (!j.is_object()) {
        wpi::errs() << "JSON error in " << file << ": not a JSON object\n";
        return false;
    }
	wpi::errs().flush();
	return true;
}
bool init(const char* fname) {

	high_resolution_clock::time_point start = high_resolution_clock::now();
	int status = 0;

	wpi::json j;
	loadJson(j, fname);
	if(j.count("ntmode") > 0) {
		try{
			std::string mode = j.at("ntmode").get<std::string>();
			if(wpi::equals_lower(mode, "server")) {
				nt::NetworkTableInstance::GetDefault().StartServer();
				std::cout << "Setup NT as SERVER" << std::endl;
			} else {
				int tnum = j.at("team").get<int>();
				std::array<std::pair<std::string_view, unsigned int>, 6> servers{{
					{SIM_ADDR, 0U}, {"172.22.11.2", 0U},
					{fmt::format("10.{}.{}.2", (tnum / 100), (tnum % 100)), 0U},
					{fmt::format("roboRIO-{}-FRC.local", tnum), 0U},
					{fmt::format("roboRIO-{}-FRC.lan", tnum), 0U},
					{fmt::format("roboRIO-{}-FRC.frc-field.local", tnum), 0U},
				}};
				nt::NetworkTableInstance::GetDefault().StartClient4(NT_IDENTITY);
				nt::NetworkTableInstance::GetDefault().SetServer(servers);
				std::cout << "Setup NT as CLIENT" << std::endl;
			}
		} catch(const wpi::json::exception& e) {
			std::cout << "NT config error in '" << fname << "': could not read 'ntmode': " << e.what() << std::endl;
			goto nt_fallback;
		}
	} else {
		nt_fallback:
#if NT_DEFAULT > 0
		nt::NetworkTableInstance::GetDefault().StartServer();
		std::cout << "Config file fallback: Setup NT as SERVER" << std::endl;
#else
		nt::NetworkTableInstance::GetDefault().StartClient4(NT_IDENTITY);
		nt::NetworkTableInstance::GetDefault().SetServer({{SIM_ADDR, "172.22.11.2"}});
		std::cout << "Config file fallback: Setup NT as CLIENT" << std::endl;
#endif
	}

	_global.disconnect_frame = cv::Mat::zeros({DEFAULT_VMODE.width, DEFAULT_VMODE.height}, CV_8UC3);
	cv::putText(
		_global.disconnect_frame, "Camera is Unavailable :(",
		cv::Point(DEFAULT_VMODE.width / 8, DEFAULT_VMODE.height / 2),
		cv::FONT_HERSHEY_SIMPLEX, 1.2, {0, 0, 255}, 4, cv::LINE_AA
	);

	_global.discon_frame_h = cs::CreateCvSource("Disconnected Frame Source", DEFAULT_VMODE, &status);
	_global.stream_h = cs::CreateMjpegServer("Viewport Stream", "", _global.next_stream_port++, &status);
	frc::CameraServer::AddServer(VideoSinkImpl(_global.stream_h));
#if DEBUG > 0
	std::cout << fmt::format("Created main stream with port {}.", _global.next_stream_port - 1) << std::endl;
#endif

	std::vector<cs::UsbCameraInfo> connections = cs::EnumerateUsbCameras(&status);

	int vid_additions = CamID::NUM_CAMERAS;
	try {
		for(const wpi::json& camera : j.at("cameras")) {
			CThread& cthr = _global.cthreads.emplace_back();
			std::string name = camera.at("name").get<std::string>();
			std::string path = camera.at("path").get<std::string>();

			cthr.camera_h = cs::CreateUsbCameraPath(fmt::format("{}_src", name), path, &status);
			cs::SetSourceConfigJson(cthr.camera_h, camera, &status);	// set to DEFAULT_VMODE if invalid
			cs::SetSourceConnectionStrategy(cthr.camera_h, CS_CONNECTION_KEEP_OPEN, &status);
			cthr.vmode = cs::GetSourceVideoMode(cthr.camera_h, &status);
			cs::SetCameraExposureManual(cthr.camera_h, DEFAULT_EXPOSURE, &status);
			cs::SetCameraWhiteBalanceAuto(cthr.camera_h, &status);
			cs::SetCameraBrightness(cthr.camera_h, DEFAULT_BRIGHTNESS, &status);

			cthr.fin_h = cs::CreateCvSink(
				fmt::format("{}_cv_in", name), &status);
			cthr.fout_h = cs::CreateCvSource(
				fmt::format("{}_cv_out", name), cthr.vmode, &status);
			cthr.view_h = cs::CreateMjpegServer(
				fmt::format("{}_view_stream", name), "", _global.next_stream_port++, &status);
			frc::CameraServer::AddServer(VideoSinkImpl(cthr.view_h));
#if DEBUG > 0
			std::cout << fmt::format("Created {} camera view stream with port {}.", name, _global.next_stream_port - 1) << std::endl;
#endif
			// cs::SetSinkSource(cthr.view_h, cthr.camera_h, &status);
			cs::SetSinkSource(cthr.view_h, cthr.fout_h, &status);		// there is some sort of bug where these streams don't switch, so start with the one more likely to be used

			for(size_t t = 0; t < CAMERA_TAGS.size(); t++) {
				if(wpi::equals_lower(name, CAMERA_TAGS[t])) {
					cthr.vid = t;
					break;
				}
			}
			if(cthr.vid < 0) { cthr.vid = vid_additions++; }

			const decltype(STATIC_CALIBRATIONS)::Cal_T* cal = nullptr;
			if(cal = findCalib(name, {cthr.vmode.width, cthr.vmode.height}, STATIC_CALIBRATIONS)) {
				std::cout << fmt::format("Found calibration for camera '{}' by name.", name) << std::endl;
				cthr.camera_matrix = cal->at(0);
				cthr.dist_matrix = cal->at(1);
#if DEBUG > 0
				std::cout << "CMatx: " << cthr.camera_matrix << std::endl;
				std::cout << "DCoefs: " << cthr.dist_matrix << std::endl;
#endif
			}

			for(cs::UsbCameraInfo& info : connections) {
				if(wpi::equals_lower(info.path, path)) {
					info.dev = -1;
#if DEBUG > 0
					std::cout << fmt::format("Camera path '{}' classified as type '{}'.", path, info.name) << std::endl;
#endif
					if(!cal && (cal = findCalib(info.name, {cthr.vmode.width, cthr.vmode.height}, STATIC_CALIBRATIONS))) {	// maybe search by path too?
						std::cout << fmt::format("Found calibration for camera '{}' by type.", name) << std::endl;
						cthr.camera_matrix = cal->at(0);
						cthr.dist_matrix = cal->at(1);
#if DEBUG > 0
						std::cout << "CMatx: " << cthr.camera_matrix << std::endl;
						std::cout << "DCoefs: " << cthr.dist_matrix << std::endl;
#endif
					}
					break;
				}
			}

			if(cal == nullptr) {
				std::cout << fmt::format("Failed to find calibration for camera '{}'.", name) << std::endl;
			}

		}
	} catch(const wpi::json::exception& e) {
		std::cout << "Config file error in '" << fname << "': Could not read camera configuration: " << e.what() << std::endl;
	}
	for(size_t i = 0; i < connections.size(); i++) {
		if(connections[i].dev >= 0 && strncmp(connections[i].name.c_str(), "bcm", 3) != 0) {
			CThread& cthr = _global.cthreads.emplace_back();
			cthr.vid = vid_additions++;
			cthr.camera_h = cs::CreateUsbCameraPath(
				fmt::format("Camera{}_src", cthr.vid), connections[i].path, &status);
			cs::SetSourceConnectionStrategy(cthr.camera_h, CS_CONNECTION_KEEP_OPEN, &status);
			cs::SetSourceVideoMode(cthr.camera_h, (cthr.vmode = DEFAULT_VMODE), &status);
			cs::SetCameraExposureManual(cthr.camera_h, DEFAULT_EXPOSURE, &status);
			cs::SetCameraWhiteBalanceAuto(cthr.camera_h, &status);
			cs::SetCameraBrightness(cthr.camera_h, DEFAULT_BRIGHTNESS, &status);

			cthr.fin_h = cs::CreateCvSink(
				fmt::format("Camera{}_cv_in", cthr.vid), &status);
			cthr.fout_h = cs::CreateCvSource(
				fmt::format("Camera{}_cv_out", cthr.vid), cthr.vmode, &status);
			cthr.view_h = cs::CreateMjpegServer(
				fmt::format("Camera{}_view_stream", cthr.vid), "", _global.next_stream_port++, &status);
			frc::CameraServer::AddServer(VideoSinkImpl(cthr.view_h));
#if DEBUG > 0
			std::cout << fmt::format("Created Camera{} view stream with port {}.", cthr.vid, _global.next_stream_port - 1) << std::endl;
#endif
			// cs::SetSinkSource(cthr.view_h, cthr.camera_h, &status);
			cs::SetSinkSource(cthr.view_h, cthr.fout_h, &status);

			const decltype(STATIC_CALIBRATIONS)::Cal_T* cal = nullptr;
			if(cal = findCalib(connections[i].name, {cthr.vmode.width, cthr.vmode.height}, STATIC_CALIBRATIONS)) {	// maybe search by path too?
				std::cout << fmt::format("Found calibration for 'Camera{}' by type.", cthr.vid) << std::endl;
				cthr.camera_matrix = cal->at(0);
				cthr.dist_matrix = cal->at(1);
#if DEBUG > 0
				std::cout << "CMatx: " << cthr.camera_matrix << std::endl;
				std::cout << "DCoefs: " << cthr.dist_matrix << std::endl;
#endif
			}
		}
	}

	std::cout << fmt::format("Cameras Available: {}", _global.cthreads.size()) << std::endl;

	_global.base_ntable = nt::NetworkTableInstance::GetDefault().GetTable("Vision");
	_global.nt.views_avail = _global.base_ntable->GetIntegerTopic("Available Outputs").GetEntry(0, NT_OPTIONS);
	_global.nt.view_id = _global.base_ntable->GetIntegerTopic("Active Camera Thread").GetEntry(0, NT_OPTIONS);
	_global.nt.ovl_verbosity = _global.base_ntable->GetIntegerTopic("Overlay Verbosity").GetEntry(0, NT_OPTIONS);
	_global.nt.exposure = _global.base_ntable->GetIntegerTopic("Camera Exposure").GetEntry(0, NT_OPTIONS);
	_global.nt.downscale = _global.base_ntable->GetIntegerTopic("Output Downscale").GetEntry(0, NT_OPTIONS);
	_global.nt.april_mode = _global.base_ntable->GetIntegerTopic("AprilTag Mode").GetEntry(-2, NT_OPTIONS);
	_global.nt.retro_mode = _global.base_ntable->GetIntegerTopic("RetroRefl Mode").GetEntry(-2, NT_OPTIONS);
	_global.nt.poses = _global.base_ntable->GetDoubleArrayTopic("Camera Pose Estimations").GetEntry({}, NT_OPTIONS);
	_global.nt.views_avail.Set(_global.cthreads.size());
	_global.nt.view_id.Set(_global.cthreads.size() > 0 ? _global.cthreads[0].vid : -1);
	_global.nt.ovl_verbosity.Set(1);
	_global.nt.exposure.Set(DEFAULT_EXPOSURE);
	_global.nt.downscale.Set(DEFAULT_DOWNSCALE);
	_global.nt.april_mode.Set(-1);
	_global.nt.retro_mode.Set(-2);
	_global.nt.poses.Set(_global.nt_pose_buffer);

	frc::SmartDashboard::init();
	frc::SmartDashboard::PutData("Vision/Stats", &_global.stats);

	_global.aprilp_params->useAruco3Detection = false;
	_global.aprilp_params->aprilTagQuadDecimate = 2.0;

	std::cout << fmt::format("Initialization completed in {}s.",
		duration<double>(high_resolution_clock::now() - start).count()) << std::endl;
	
	return true;
}

void _update(CThread& ctx) {
	int status = 0;
	int apmode = _global.nt.april_mode.Get();
	bool downscale = _global.nt.downscale.Get() > 1;
	bool overlay = _global.nt.ovl_verbosity.Get() > 0;
	bool outputting = ctx.vid == _global.nt.view_id.Get();
	bool connected = cs::IsSourceConnected(ctx.camera_h, &status);
	ctx.procm = 1 * (((apmode == -1) && outputting) || apmode == ctx.vid);
	bool enable = connected && ((overlay && outputting) || downscale || ctx.procm);

	if(_global.state.view_updated || _global.state.vrbo_updated || _global.state.dscale_updated) {
		if(connected) {
			if(overlay || downscale) {
				cs::SetSinkSource(ctx.view_h, ctx.fout_h, &status);
				if(outputting) cs::SetSinkSource(_global.stream_h, ctx.fout_h, &status);
			} else {
				cs::SetSinkSource(ctx.view_h, ctx.camera_h, &status);
				if(outputting) cs::SetSinkSource(_global.stream_h, ctx.camera_h, &status);
			}
		} else {
			cs::SetSinkSource(ctx.view_h, _global.discon_frame_h, &status);
			if(outputting) cs::SetSinkSource(_global.stream_h, _global.discon_frame_h, &status);
		}
	}
	if(_global.state.exposure_updated) {
		cs::SetCameraExposureManual(ctx.camera_h, _global.nt.exposure.Get(), &status);
	}
	if(enable) {	// add other states to compute (ex. isproc, isactive) in the future too
		if(!ctx.proc.joinable()) {
			ctx.link_state = true;
			ctx.proc = std::thread(_worker, std::ref(ctx));
		}
	} else if(ctx.proc.joinable()) {
		ctx.link_state = false;
		ctx.proc.join();
	}
	if(outputting && !connected) {
		cs::PutSourceFrame(_global.discon_frame_h, _global.disconnect_frame, &status);
	}
}
void _worker(CThread& ctx) {

	int status = 0;
	cs::SetSinkSource(ctx.fin_h, ctx.camera_h, &status);

	high_resolution_clock::time_point tp = high_resolution_clock::now();
	int verbosity, downscale;
	cv::Size fsz{ctx.vmode.width, ctx.vmode.height};

	for(;ctx.link_state && _global.state.program_enable;) {

		cs::GrabSinkFrame(ctx.fin_h, ctx.frame, &status);
		high_resolution_clock::time_point t = high_resolution_clock::now();
		ctx.ftime = duration<float>(t - tp).count();
		tp = t;
		verbosity = _global.nt.ovl_verbosity.Get();
		downscale = _global.nt.downscale.Get();
		// if(verbosity <= 0 && downscale <= 1) {
		// 	cs::PutSourceFrame(ctx.fout_h, ctx.frame, &status);		// keep latency as short as possible
		// }
		switch(ctx.procm) {
			case 0:
			default:{	// none
				if(verbosity > 3) {
					if(ctx.vproc_buffer->fbuff.size() != fsz) {
						ctx.vproc_buffer->fbuff = cv::Mat(fsz, CV_8UC1);
					}
					high_resolution_clock::time_point s = high_resolution_clock::now();
					// neon_deinterlace_wst(
					// 	ctx.frame, ctx.vproc_buffer->fbuff, vs2::BGR::GREEN
					// );
					memcpy_deinterlace_wst_asm(ctx.frame.data, ctx.vproc_buffer->fbuff.data, fsz.area(), 1, 0x7F, 0x7F, 0x00);
					float dt = duration<float>(high_resolution_clock::now() - s).count();
					cv::cvtColor(ctx.vproc_buffer->fbuff, ctx.frame, cv::COLOR_GRAY2BGR);
					cv::putText(
						ctx.frame, fmt::format("NEON Deinterlace WST: {}ns", dt * 1e6),
						cv::Point(5, 300), cv::FONT_HERSHEY_DUPLEX,
						0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA
					);
				}
				if(verbosity > 0) {
					cv::putText(
						ctx.frame, std::to_string(1.f / ctx.ftime),
						cv::Point(5, 20), cv::FONT_HERSHEY_DUPLEX,
						0.65, cv::Scalar(0, 255, 0), 1, cv::LINE_AA
					);
				}
				if(downscale > 1) {
					cv::Size f = fsz / downscale;
					if(ctx.buff.size() != f) {
						ctx.buff = cv::Mat(f, CV_8UC3);
					}
					cv::resize(ctx.frame, ctx.buff, f);
					cs::PutSourceFrame(ctx.fout_h, ctx.buff, &status);
				} else {
					cs::PutSourceFrame(ctx.fout_h, ctx.frame, &status);
				}
			}
			case 1: {	// apriltag
				if(ctx.vproc_buffer == nullptr) {
					ctx.vproc_buffer = new CThread::ProcBuff{};
				}
				ctx.vproc_buffer->corners.clear();
				ctx.vproc_buffer->ids.clear();
				// resize
				high_resolution_clock::time_point detect_beg = high_resolution_clock::now();
				_ap_detect_aruco(
					ctx.frame, ctx.vproc_buffer->corners, ctx.vproc_buffer->ids);
				float dt = duration<float>(high_resolution_clock::now() - detect_beg).count();

				if(verbosity > 1) {
					if(ctx.vproc_buffer->ids.size() > 0) {
						cv::aruco::drawDetectedMarkers(
							ctx.frame, ctx.vproc_buffer->corners, ctx.vproc_buffer->ids);
					}
					cv::putText(
						ctx.frame, fmt::format("Detection (ms): {}", dt * 1000.0),
						cv::Point(5, 40), cv::FONT_HERSHEY_DUPLEX,
						0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA
					);
				}
				if(verbosity > 0) {
					cv::putText(
						ctx.frame, std::to_string(1.f / ctx.ftime),
						cv::Point(5, 20), cv::FONT_HERSHEY_DUPLEX,
						0.65, cv::Scalar(0, 255, 0), 1, cv::LINE_AA
					);
				}
				if(downscale > 1) {
					cv::Size f = fsz / downscale;
					if(ctx.buff.size() != f) {
						ctx.buff = cv::Mat(f, CV_8UC3);
					}
					cv::resize(ctx.frame, ctx.buff, f);
					cs::PutSourceFrame(ctx.fout_h, ctx.buff, &status);
				} else {
					cs::PutSourceFrame(ctx.fout_h, ctx.frame, &status);
				}

				// clear tvecs, rvecs?
				if(ctx.vproc_buffer->ids.size() > 0) {
					std::vector<frc::Pose3d> poses;
					_ap_estimate_aruco(
						ctx.vproc_buffer->corners, ctx.vproc_buffer->ids,
						ctx.camera_matrix, ctx.dist_matrix, poses/*,
						ctx.vproc_buffer->tvecs, ctx.vproc_buffer->rvecs*/);

					double* d = reinterpret_cast<double*>(poses.data());
					_global.nt.poses.Set(std::span<double>(d, d + (poses.size() * (sizeof(frc::Pose3d) / sizeof(double)))));
				}
			}
		}

	}

}
void _shutdown(CThread& ctx) {
	int status = 0;
	if(ctx.proc.joinable()) {
		ctx.link_state = false;
		ctx.proc.join();
	}
	if(ctx.vproc_buffer != nullptr) {
		delete ctx.vproc_buffer;
	}
	cs::ReleaseSource(ctx.camera_h, &status);
	cs::ReleaseSource(ctx.fout_h, &status);
	cs::ReleaseSink(ctx.fin_h, &status);
}



// frc::Pose3d toPose3d(
// 	const CThread::ProcBuff::PVec& tvec,
// 	const CThread::ProcBuff::PVec& rvec
// ) {
// 	// cv::Mat_<float> R, t;
// 	// cv::Rodrigues(rvec, R);
// 	// R = R.t();
// 	// t = -R * tvec;

// 	frc::Vectord<3> rv{ +rvec.at<float>(2, 0), -rvec.at<float>(0, 0), -rvec.at<float>(1, 0) };

// 	// return frc::Pose3d(
// 	// 	units::inch_t{ +t.at<float>(2, 0) },
// 	// 	units::inch_t{ -t.at<float>(0, 0) },
// 	// 	units::inch_t{ -t.at<float>(1, 0) },
// 	// 	// units::inch_t{ +t.at<float>(0, 0) },
// 	// 	// units::inch_t{ +t.at<float>(1, 0) },
// 	// 	// units::inch_t{ -t.at<float>(2, 0) },
// 	// 	frc::Rotation3d{ rv, units::radian_t{ rv.norm() } }
// 	// );

// 	frc::Translation3d t3{
// 		units::inch_t{ tvec.at<float>(2, 0) },
// 		units::inch_t{ -tvec.at<float>(0, 0) },
// 		units::inch_t{ -tvec.at<float>(1, 0) } };
// 	frc::Rotation3d r3{ rv, units::radian_t{ rv.norm() } };
// 	frc::Transform3d t = frc::Transform3d{ t3, r3 }.Inverse();
// 	return frc::Pose3d{ t.Translation(), t.Rotation() };
// }
// std::span<frc::Pose3d> toPose3d(
// 	std::vector<frc::Pose3d>& poses,
// 	const std::vector<CThread::ProcBuff::PVec>& tvecs,
// 	const std::vector<CThread::ProcBuff::PVec>& rvecs
// ) {
// 	size_t beg = poses.size();
// 	for(size_t i = 0; i < tvecs.size(); i++) {
// 		poses.emplace_back(toPose3d(tvecs[i], rvecs[i]));
// 	}
// 	return std::span<frc::Pose3d>{ poses.begin() + beg, poses.end() };
// }

template<typename f>
cv::Point3_<f> wpiToCv(const cv::Point3_<f>& p) {
	return cv::Point3_<f>( -p.y, -p.z, p.x );
}
template<typename u = units::inch_t>
frc::Translation3d cvToWpi_T(const cv::Mat1f& tvec) {
	return frc::Translation3d(
		u{ +tvec.at<float>(2, 0) },
		u{ -tvec.at<float>(0, 0) },
		u{ -tvec.at<float>(1, 0) }
	);
}
frc::Rotation3d cvToWpi_R(const cv::Mat1f& rvec) {
	frc::Vectord<3> rv{
		+rvec.at<float>(2, 0),
		-rvec.at<float>(0, 0),
		-rvec.at<float>(1, 0)
	};
	return frc::Rotation3d( rv, units::radian_t{ rv.norm() } );
}
template<typename u = units::inch_t>
frc::Pose3d cvToWpi_P(const cv::Mat1f& tvec, const cv::Mat1f& rvec) {
	return frc::Pose3d{
		cvToWpi_T<u>(tvec),
		cvToWpi_R(rvec)
	};
}
template<typename u = units::inch_t>
frc::Transform3d cvToWpi(const cv::Mat1f& tvec, const cv::Mat1f& rvec) {
	return frc::Transform3d{
		cvToWpi_T<u>(tvec),
		cvToWpi_R(rvec)
	};
}

void _ap_detect_aruco(
	const cv::Mat& frame,
	std::vector<std::vector<cv::Point2f>>& corners,
	std::vector<int32_t>& ids
) {
	cv::aruco::detectMarkers(
		frame, _global.aprilp_field->dictionary,
		corners, ids, _global.aprilp_params
	);
}

int _ap_estimate_aruco(
	std::vector<std::vector<cv::Point2f>>& corners, std::vector<int32_t>& ids,
	cv::InputArray cmatx, cv::InputArray cdist,
	std::vector<frc::Pose3d>& estimations/*,*/

	// std::vector<cv::Mat1f>& _tvecs, std::vector<cv::Mat1f>& _rvecs,
	// std::vector<cv::Point3f>& _obj_points, std::vector<cv::Point2f>& _img_points
) {
	CV_Assert(corners.size() == ids.size());

	std::vector<cv::Mat1f> _tvecs, _rvecs;

	size_t ndetected = ids.size();
	if(ndetected == 0) { return 0; }
	else if(ndetected == 1) {
		cv::solvePnPGeneric(
			GENERIC_TAG_CORNERS, corners.at(0),
			cmatx, cdist, _rvecs, _tvecs,
			false, cv::SOLVEPNP_IPPE_SQUARE
		);

		int id = ids.at(0);

		frc::Pose3d tag = _global.aprilp_field_poses.GetTagPose(id).value();
		for(size_t i = 0; i < _tvecs.size(); i++) {
			frc::Transform3d c2tag = cvToWpi(_tvecs[i], _rvecs[i]);
			estimations.push_back(tag.TransformBy(c2tag.Inverse()));
		}
		return _tvecs.size();
	}

	CV_Assert(_global.aprilp_field->ids.size() == _global.aprilp_field->objPoints.size());

	std::vector<cv::Point3f> _obj_points;
	std::vector<cv::Point2f> _img_points;
	// _obj_points.clear();
	// _img_points.clear();
	_obj_points.reserve(ndetected * 4);
	_img_points.reserve(ndetected * 4);
	
	for(size_t i = 0; i < ndetected; i++) {
		int id = ids.at(i);
		for(size_t j = 0; j < _global.aprilp_field->ids.size(); j++) {
			if(id == _global.aprilp_field->ids[j]) {
				_img_points.insert(_img_points.end(), corners.at(i).begin(), corners.at(i).end());
				for(int p = 3; p >= 0; p--) {	// reverse the order such as to flip the points vertically
					_obj_points.push_back(wpiToCv(_global.aprilp_field->objPoints[j][p]));
				}
			}
		}
	}

	CV_Assert(_img_points.size() == _obj_points.size());

	cv::solvePnPGeneric(
		_obj_points, _img_points,
		cmatx, cdist, _rvecs, _tvecs,
		false, cv::SOLVEPNP_ITERATIVE
	);

	for(size_t i = 0; i < _tvecs.size(); i++) {
		frc::Transform3d f2cam = cvToWpi(_tvecs[i], _rvecs[i]).Inverse();
		estimations.emplace_back(f2cam.Translation(), f2cam.Rotation());
	}

	return _tvecs.size();

}
