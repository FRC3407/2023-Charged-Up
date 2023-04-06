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





class VideoSinkImpl : public cs::VideoSink {
public:
	inline VideoSinkImpl(CS_Sink h) : VideoSink(h) {}	// this constructor is protected so we have to subclass to use it publicly
};

class Stats : public wpi::Sendable, public CoreStats {
public:
	inline Stats(bool all_cores = false) : CoreStats(all_cores) {
		this->april_profile.resize(3);
		this->retro_profile.resize(2);
	}

	virtual void InitSendable(wpi::SendableBuilder& b) override {
		b.AddFloatProperty("Core temp", [this](){ return this->temp(); }, nullptr);
		b.AddFloatProperty("Utilization", [this](){ return this->fromLast(); }, nullptr);
		// b.AddFloatProperty("Avg FTime", [this](){ return (float)this->ftime_avg; }, nullptr);
		b.AddDoubleArrayProperty("Camera Thread FTimes", [this](){ return this->ftimes; }, nullptr);
		b.AddDoubleArrayProperty("April VPipe Profiling", [this](){ return this->april_profile; }, nullptr);
		// b.AddDoubleArrayProperty("Retro VPipe Profiling", [this](){ return this->retro_profile; }, nullptr);
	}

	// std::atomic<float> ftime_avg{};
	std::vector<double>
		ftimes{}, april_profile{}, retro_profile{};

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
		view_h(t.view_h),
		link_state(t.link_state.load()),
		vpmode(t.vpmode.load()),
		vproc(std::move(t.vproc)),
		frame(std::move(t.frame)),
		dframe(std::move(t.dframe)),
		ftime(t.ftime.load()) {}

	cv::Mat1f
		camera_matrix{ DEFAULT_CAM_MATX },
		dist_matrix{ DEFAULT_CAM_MATX };
	cs::VideoMode vmode;

	int vid{-1};
	CS_Source camera_h, fout_h;
	CS_Sink fin_h, view_h;

	std::atomic<bool> link_state{true};
	std::atomic<int> vpmode{0};
	std::thread vproc;

	cv::Mat frame, dframe, sframe;
	std::atomic<float> ftime{0.f};

};
void _update(CThread&);	// these are like instance methods
void _worker(CThread&);
void _shutdown(CThread&);





namespace util {

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

	template<typename u = units::inch_t>
	frc::Pose3d cvToWpiInv_P(const cv::Mat1f& tvec, const cv::Mat1f& rvec) {
		cv::Mat1f R, t;
		cv::Rodrigues(rvec, R);
		R = R.t();
		t = -R * tvec;
		return cvToWpi_P<u>(t, rvec);
	}
	template<typename u = units::inch_t>
	frc::Transform3d cvToWpiInv(const cv::Mat1f& tvec, const cv::Mat1f& rvec) {
		cv::Mat1f R, t;
		cv::Rodrigues(rvec, R);
		R = R.t();
		t = -R * tvec;
		return cvToWpi<u>(t, rvec);
	}

};

void _ap_detect_aruco(
	const cv::Mat& frame,
	std::vector<std::vector<cv::Point2f>>& corners, std::vector<int32_t>& ids
);
int _ap_estimate_aruco(
	std::vector<std::vector<cv::Point2f>>& corners, std::vector<int32_t>& ids,
	cv::InputArray cmatx, cv::InputArray cdist,
	std::vector<frc::Pose3d>& estimations
);
int _ap_estimate_aruco(
	std::vector<std::vector<cv::Point2f>>& corners, std::vector<int32_t>& ids,
	cv::InputArray cmatx, cv::InputArray cdist,
	std::vector<frc::Pose3d>& estimations,
	std::vector<cv::Mat1f>& _tvecs, std::vector<cv::Mat1f>& _rvecs,
	std::vector<cv::Point3f>& _obj_points, std::vector<cv::Point2f>& _img_points
);





/* Global Storage */
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
	struct {
		std::thread april_worker, retro_worker;
		std::atomic<int> april_link{0}, retro_link{0};
		struct {
			std::vector<std::vector<cv::Point2f>> tag_corners;
			std::vector<int32_t> tag_ids;
			std::vector<cv::Point2f> img_points;
			std::vector<cv::Point3f> obj_points;
			std::vector<cv::Mat1f> tvecs, rvecs;
			std::vector<frc::Pose3d> estimations;
		} apbuff;
		// struct {} rtbuff;
	} vpp;	// 'Vision Processing Pipeline'

} _global;

void _april_worker_inst(CThread&);
void _retro_worker_inst(CThread&);
void _april_worker(CThread&);
void _retro_worker(CThread&);

bool init(const char* f = FRC_CONFIG);




int main(int argc, char** argv) {

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
		// float a = 0;
		for(CThread& t : _global.cthreads) {
			_update(t);
			// a += t.ftime;
			_global.stats.ftimes[i] = t.ftime;
			i++;
		}
		// _global.stats.ftime_avg = a / i;

		frc::SmartDashboard::UpdateValues();

		std::this_thread::sleep_for(100ms);
	}

	// shutdown
	{
		std::cout << "\nShutting down threads..." << std::endl;
		for(CThread& t : _global.cthreads) { _shutdown(t); }
		if(_global.vpp.april_worker.joinable()) { _global.vpp.april_worker.join(); }
		if(_global.vpp.retro_worker.joinable()) { _global.vpp.retro_worker.join(); }
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

	_global.stats.ftimes.resize(_global.cthreads.size());
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
	int rflmode = _global.nt.retro_mode.Get();
	bool downscale = _global.nt.downscale.Get() > 1;
	bool overlay = _global.nt.ovl_verbosity.Get() > 0;
	bool outputting = ctx.vid == _global.nt.view_id.Get();
	bool connected = cs::IsSourceConnected(ctx.camera_h, &status);
	ctx.vpmode = (
		((int)(((apmode == -1) && outputting) || apmode == ctx.vid) << 0) |		// first bit is apmode, second is reflmode
		((int)(((rflmode == -1) && outputting) || rflmode == ctx.vid) << 1)
	);
	bool enable = connected && ((overlay && outputting) || downscale || ctx.vpmode);

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
		if(!ctx.vproc.joinable()) {
			ctx.link_state = true;
			ctx.vproc = std::thread(_worker, std::ref(ctx));
		}
	} else if(ctx.vproc.joinable()) {
		ctx.link_state = false;
		ctx.vproc.join();
	}
	if(outputting && !connected) {
		cs::PutSourceFrame(_global.discon_frame_h, _global.disconnect_frame, &status);
	}
}
void _worker(CThread& ctx) {

	int status = 0;
	cs::SetSinkSource(ctx.fin_h, ctx.camera_h, &status);

	high_resolution_clock::time_point tp = high_resolution_clock::now();
	cv::Size fsz{ctx.vmode.width, ctx.vmode.height};
	int verbosity, downscale;

	for(;ctx.link_state && _global.state.program_enable;) {

		cs::GrabSinkFrame(ctx.fin_h, ctx.frame, &status);
		high_resolution_clock::time_point t = high_resolution_clock::now();
		ctx.ftime = duration<float>(t - tp).count();
		tp = t;

		verbosity = _global.nt.ovl_verbosity.Get();
		downscale = _global.nt.downscale.Get();

		if(ctx.vpmode) { ctx.frame.copyTo(ctx.sframe); }
		if(ctx.vpmode & 0b01) {		// apriltag

			if(_global.vpp.april_link > 1 && _global.vpp.april_worker.joinable()) {
				_global.vpp.april_worker.join();
				_global.vpp.april_link = 0;
			}
			if(_global.vpp.april_link == 0) {	// if the thread is stopped
				if(verbosity > 1 && _global.vpp.apbuff.tag_ids.size() > 0) {
					cv::aruco::drawDetectedMarkers(ctx.frame, _global.vpp.apbuff.tag_corners, _global.vpp.apbuff.tag_ids);
					cv::putText(ctx.frame,
						fmt::format("Detection time: {:.3f}ms", _global.stats.april_profile[0]),
						cv::Point(5, 40), cv::FONT_HERSHEY_DUPLEX,
						0.5, {0, 255, 0}, 1, cv::LINE_AA
					);
					cv::putText(ctx.frame,
						fmt::format("Estimation time: {:.3f}ms", _global.stats.april_profile[1]),
						cv::Point(5, 55), cv::FONT_HERSHEY_DUPLEX,
						0.5, {0, 255, 0}, 1, cv::LINE_AA
					);
					cv::putText(ctx.frame,
						fmt::format("Total thread time: {:.3f}ms", _global.stats.april_profile[2]),
						cv::Point(5, 70), cv::FONT_HERSHEY_DUPLEX,
						0.5, {0, 255, 0}, 1, cv::LINE_AA
					);
				}
				_global.vpp.april_worker = std::thread(_april_worker, std::ref(ctx));
			}

		}
		if(ctx.vpmode & 0b10) {		// retroreflective

		}

		if(verbosity > 0) {
			cv::putText(
				ctx.frame, fmt::format("{:.1f}", 1.f / ctx.ftime),
				cv::Point(5, 20), cv::FONT_HERSHEY_DUPLEX,
				0.65, {0, 255, 0}, 1, cv::LINE_AA
			);
		}
		if(downscale > 1) {
			cv::Size f = fsz / downscale;
			if(ctx.dframe.size() != f) {
				ctx.dframe = cv::Mat(f, CV_8UC3);
			}
			cv::resize(ctx.frame, ctx.dframe, f);
			cs::PutSourceFrame(ctx.fout_h, ctx.dframe, &status);
		} else {
			cs::PutSourceFrame(ctx.fout_h, ctx.frame, &status);
		}

	}

}
void _shutdown(CThread& ctx) {
	int status = 0;
	if(ctx.vproc.joinable()) {
		ctx.link_state = false;
		ctx.vproc.join();
	}
	cs::ReleaseSource(ctx.camera_h, &status);
	cs::ReleaseSource(ctx.fout_h, &status);
	cs::ReleaseSink(ctx.fin_h, &status);
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
	std::vector<frc::Pose3d>& estimations
) {
	std::vector<cv::Mat1f> _tvecs, _rvecs;
	std::vector<cv::Point3f> _obj_points;
	std::vector<cv::Point2f> _img_points;
	_ap_estimate_aruco(corners, ids, cmatx, cdist, estimations, _tvecs, _rvecs, _obj_points, _img_points);
}
int _ap_estimate_aruco(
	std::vector<std::vector<cv::Point2f>>& corners, std::vector<int32_t>& ids,
	cv::InputArray cmatx, cv::InputArray cdist,
	std::vector<frc::Pose3d>& estimations,

	std::vector<cv::Mat1f>& _tvecs, std::vector<cv::Mat1f>& _rvecs,
	std::vector<cv::Point3f>& _obj_points, std::vector<cv::Point2f>& _img_points
) {
	CV_Assert(corners.size() == ids.size());

	// std::vector<cv::Mat1f> _tvecs, _rvecs;

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
			// frc::Transform3d c2tag = cvToWpi(_tvecs[i], _rvecs[i]);
			// estimations.push_back(tag.TransformBy(c2tag.Inverse()));
			estimations.emplace_back(tag.TransformBy(util::cvToWpiInv(_tvecs[i], _rvecs[i])));
		}
		return _tvecs.size();
	}

	CV_Assert(_global.aprilp_field->ids.size() == _global.aprilp_field->objPoints.size());

	// std::vector<cv::Point3f> _obj_points;
	// std::vector<cv::Point2f> _img_points;
	_obj_points.clear();
	_img_points.clear();
	_obj_points.reserve(ndetected * 4);
	_img_points.reserve(ndetected * 4);
	
	for(size_t i = 0; i < ndetected; i++) {
		int id = ids.at(i);
		for(size_t j = 0; j < _global.aprilp_field->ids.size(); j++) {
			if(id == _global.aprilp_field->ids[j]) {
				_img_points.insert(_img_points.end(), corners.at(i).begin(), corners.at(i).end());
				for(int p = 3; p >= 0; p--) {	// reverse the order such as to flip the points vertically
					_obj_points.push_back(util::wpiToCv(_global.aprilp_field->objPoints[j][p]));
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
		// frc::Transform3d f2cam = cvToWpi(_tvecs[i], _rvecs[i]).Inverse();
		// estimations.emplace_back(f2cam.Translation(), f2cam.Rotation());
		estimations.emplace_back(util::cvToWpiInv_P(_tvecs[i], _rvecs[i]));
	}

	return _tvecs.size();

}


void _april_worker_inst(CThread& target) {

	decltype(_global.vpp.apbuff)& _buff = _global.vpp.apbuff;

	high_resolution_clock::time_point p, beg = high_resolution_clock::now();

	_global.vpp.april_link = 1;
	_buff.tag_corners.clear();
	_buff.tag_ids.clear();

	p = high_resolution_clock::now();
	_ap_detect_aruco(target.sframe, _buff.tag_corners, _buff.tag_ids);
	_global.stats.april_profile[0] = duration<double>(high_resolution_clock::now() - p).count();

	_buff.estimations.clear();		// may do something with these later, like seed the new estimations?
	p = high_resolution_clock::now();
	_ap_estimate_aruco(
		_buff.tag_corners, _buff.tag_ids, target.camera_matrix, target.dist_matrix, _buff.estimations,
		_buff.tvecs, _buff.rvecs, _buff.obj_points, _buff.img_points);
	_global.stats.april_profile[1] = duration<double>(high_resolution_clock::now() - p).count();

	double* d = reinterpret_cast<double*>(_buff.estimations.data());
	_global.nt.poses.Set(std::span<double>(d, d + (_buff.estimations.size() * (sizeof(frc::Pose3d) / sizeof(double)))));

	_global.vpp.april_link = 2;

	_global.stats.april_profile[2] = duration<double>(high_resolution_clock::now() - beg).count();

}
void _retro_worker_inst(CThread& target) {}

void _april_worker(CThread& target) {

	_april_worker_inst(target);		// may convert this into a loop with state checking later

}
void _retro_worker(CThread& target) {

	_retro_worker_inst(target);		// '''

}
