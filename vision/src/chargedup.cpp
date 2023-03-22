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
#include <networktables/NetworkTable.h>
#include <networktables/IntegerTopic.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Pose3d.h>
#include <frc/apriltag/AprilTagDetector.h>
#include <cameraserver/CameraServer.h>

#include <libpixyusb2.h>

#include <core/calib.h>

#include <cpp-tools/src/sighandle.h>
#include <cpp-tools/src/unix/stats2.h>
#include <cpp-tools/src/unix/stats2.cpp>

#include "field.h"


#define DLOG(x) std::cout << (x) << std::endl;
#define DEBUG 0

#define FRC_CONFIG "/boot/frc.json"
#define NT_IDENTITY "Vision RPI"
#define SIM_ADDR "192.168.0.8"
#define DEBUG_VIEW 0
#define NT_DEFAULT 0
#define SEND_EXTRA_POSE_ESTIMATIONS 0

using namespace std::chrono_literals;
using namespace std::chrono;

enum CamID {
	FWD_CAMERA,
	ARM_CAMERA,
	TOP_CAMERA,
	// PIXY2,
	START_ADDITIONAL
};
static const nt::PubSubOptions
	NT_OPTIONS = { .periodic = 1.0 / 30.0 };
static const std::array<const char*, (size_t)CamID::START_ADDITIONAL>
	CAMERA_TAGS{ "forward", "arm", "top"/*, "pixy2"*/ };
static const std::array<frc::Pose3d, (size_t)CamID::START_ADDITIONAL>
	CAMERA_POSES{		// camera poses relative to robot center position
		frc::Pose3d{},
		frc::Pose3d{},
		frc::Pose3d{}
	};
static const cs::VideoMode
	DEFAULT_VMODE{ cs::VideoMode::kMJPEG, 640, 480, 30 };
static const int
	DEFAULT_EXPOSURE = 40,
	DEFAULT_WBALANCE = -1,
	DEFAULT_BRIGHTNESS = 50;
static const CalibSList
	STATIC_CALIBRATIONS{
		{
			{ "lifecam_hd3000", {
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
				} }
			} },
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
	}

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
	frc::Pose3d
		*robot_camera_pose{ nullptr };
	cs::VideoMode vmode;

	struct ProcBuff {
		using PVec = std::array<float, 3>;

		std::vector<std::vector<cv::Point2f> > corners;
		std::vector<int32_t> ids;
		std::vector<PVec> tvecs, rvecs;
		cv::Mat fbuff;
	} *vproc_buffer{nullptr};
	cv::Mat frame;

	int vid{-1};
	CS_Source camera_h, fout_h;
	CS_Sink fin_h;

	std::atomic<bool> link_state{true};
	std::atomic<int> procm{0};
	std::thread proc;

};
bool _update(CThread&);	// these are like instance methods
void _worker(CThread&);
void _shutdown(CThread&);


bool init(const char* f = FRC_CONFIG);

struct {
	system_clock::time_point start_time;
	struct {
		std::atomic<bool>
			program_enable{true},
			view_updated{false},
			vrbo_updated{false}
		;
	} state;
	Stats stats{};

	std::shared_ptr<nt::NetworkTable> base_ntable;
	struct {
		nt::IntegerEntry
			views_avail,
			view_id,		// active camera id
			ovl_verbosity,	// overlay verbosity - 0 for off, >0 for increasing verbosity outputs
			april_mode		// apriltag detection mode - 0 for off, -1 for all threads, -2 for active thread, >0 for specific thread
		;
	} nt;

	int next_stream_port = 1181;
	std::vector<CThread> cthreads;
	cv::Mat disconnect_frame;
	// Pixy2 pixycam;
	CS_Source discon_frame_h;
	CS_Sink stream_h;

	struct {
		const cv::Ptr<cv::aruco::DetectorParameters> params{cv::aruco::DetectorParameters::create()};
		const cv::Ptr<cv::aruco::Board> field{::FIELD_2023};
		const frc::AprilTagDetector wpi_detector{};

		std::vector<frc::Pose3d> robot_est_queue, raw_est_queue;
		std::mutex robot_queue_rw, raw_queue_rw;
		std::thread queue;
		std::atomic<bool> link_state;
	} aprilpose;

} _global;
void _ap_update();
void _ap_worker();
void _ap_detect_aruco(const cv::Mat&, std::vector<std::vector<cv::Point2f>>&, std::vector<int32_t>&);
void _ap_detect_wpi(const cv::Mat&, frc::AprilTagDetector::Results&);
int _ap_estimate_aruco(cv::InputArrayOfArrays&, cv::InputArray&, cv::InputArray, cv::InputArray, std::vector<CThread::ProcBuff::PVec>&, std::vector<CThread::ProcBuff::PVec>&);

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
	int last_vid = -1;
	int last_vrb = 0;
	for(;_global.state.program_enable;) {

		int vid = _global.nt.view_id.Get();
		int vrb = _global.nt.ovl_verbosity.Get();
		_global.state.view_updated = (vid != last_vid);
		_global.state.vrbo_updated = (vrb != last_vrb);
		last_vid = vid;
		last_vrb = vrb;

		for(CThread& t : _global.cthreads) {
			_update(t);
		}
		_ap_update();

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
		// _global.pixycam.m_link.close();
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

	frc::SmartDashboard::init();
	frc::SmartDashboard::PutData("Vision/Stats", &_global.stats);

	_global.disconnect_frame = cv::Mat::zeros({DEFAULT_VMODE.width, DEFAULT_VMODE.height}, CV_8UC3);
	cv::putText(
		_global.disconnect_frame, "Camera is Unavailable :(",
		cv::Point(DEFAULT_VMODE.width / 8, DEFAULT_VMODE.height / 2),
		cv::FONT_HERSHEY_SIMPLEX, 1.2, {0, 0, 255}, 4, cv::LINE_AA
	);

	_global.discon_frame_h = cs::CreateCvSource("Disconnected Frame Source", DEFAULT_VMODE, &status);
	_global.stream_h = cs::CreateMjpegServer("Viewport Stream", "", _global.next_stream_port++, &status);
	frc::CameraServer::AddServer(VideoSinkImpl(_global.stream_h));

	// status = _global.pixycam.init();
	// switch(status) {
	// 	default:
	// 	case PIXY_RESULT_OK: {
	// 		std::cout << "Pixy init successful." << std::endl;
	// 		_global.pixycam.getVersion();
	// 		//_global.pixycam.version->print();
	// 		break;
	// 	}
	// 	case PIXY_RESULT_ERROR: {
	// 		std::cout << "Pixy init fail - general error." << std::endl;
	// 		break;
	// 	}
	// 	case PIXY_RESULT_BUSY: {
	// 		std::cout << "Pixy init fail - no new data (busy)." << std::endl;
	// 		break;
	// 	}
	// 	case PIXY_RESULT_CHECKSUM_ERROR: {
	// 		std::cout << "Pixy init fail - checksum error." << std::endl;
	// 		break;
	// 	}
	// 	case PIXY_RESULT_TIMEOUT: {
	// 		std::cout << "Pixy init fail - timeout." << std::endl;
	// 		break;
	// 	}
	// 	case PIXY_RESULT_BUTTON_OVERRIDE: {
	// 		std::cout << "Pixy init fail - user button override." << std::endl;
	// 		break;
	// 	}
	// 	case PIXY_RESULT_PROG_CHANGING: {
	// 		std::cout << "Pixy init fail - program is changing." << std::endl;
	// 		break;
	// 	}
	// }

	std::vector<cs::UsbCameraInfo> connections = cs::EnumerateUsbCameras(&status);

	int vid_additions = CamID::START_ADDITIONAL;
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

			cthr.fin_h = cs::CreateCvSink(fmt::format("{}_cv_in", name), &status);
			cthr.fout_h = cs::CreateCvSource(fmt::format("{}_cv_out", name), cthr.vmode, &status);
			
			for(size_t t = 0; t < CAMERA_TAGS.size(); t++) {
				if(wpi::equals_lower(name, CAMERA_TAGS[t])) {
					cthr.vid = t;
					cthr.robot_camera_pose = &CAMERA_POSES[t];
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

			// calibrations
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
		}
	}

	std::cout << fmt::format("Cameras Available: {}", _global.cthreads.size()) << std::endl;

	_global.base_ntable = nt::NetworkTableInstance::GetDefault().GetTable("Vision");
	_global.nt.views_avail = _global.base_ntable->GetIntegerTopic("Available Outputs").GetEntry(0, NT_OPTIONS);
	_global.nt.view_id = _global.base_ntable->GetIntegerTopic("Active Camera Thread").GetEntry(0, NT_OPTIONS);
	_global.nt.ovl_verbosity = _global.base_ntable->GetIntegerTopic("Overlay Verbosity").GetEntry(0, NT_OPTIONS);
	_global.nt.april_mode = _global.base_ntable->GetIntegerTopic("AprilTag Mode").GetEntry(0, NT_OPTIONS);
	_global.nt.views_avail.Set(_global.cthreads.size());
	_global.nt.view_id.Set(_global.cthreads.size() > 0 ? _global.cthreads[0].vid : -1);
	_global.nt.ovl_verbosity.Set(1);
	_global.nt.april_mode.Set(0);

	_global.aprilpose.wpi_detector.AddFamily(FRC_DICT_NAME);

	std::cout << fmt::format("Initialization completed in {}s.",
		duration<double>(high_resolution_clock::now() - start).count()) << std::endl;
	
	return true;
}

bool _update(CThread& ctx) {
	int status = 0;
	bool outputting = ctx.vid == _global.nt.view_id.Get();
	bool connected = cs::IsSourceConnected(ctx.camera_h, &status);
	bool overlay = _global.nt.ovl_verbosity.Get() > 0;
	int apmode = _global.nt.april_mode.Get();
	ctx.procm = 1 * (apmode == -2 || (apmode == -1) && outputting || apmode == ctx.vid);
	bool enable = connected && ((overlay && outputting) || ctx.procm);

	if(outputting && (_global.state.view_updated || _global.state.vrbo_updated)) {
		if(connected) {
			if(overlay) {
				cs::SetSinkSource(_global.stream_h, ctx.fout_h, &status);
			} else {
				cs::SetSinkSource(_global.stream_h, ctx.camera_h, &status);
			}
		} else {
			cs::SetSinkSource(_global.stream_h, _global.discon_frame_h, &status);
		}
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
	return !enable;
}
void _worker(CThread& ctx) {

	int status = 0;
	cs::SetSinkSource(ctx.fin_h, ctx.camera_h, &status);

	high_resolution_clock::time_point tp = high_resolution_clock::now();
	int verbosity;

	for(;ctx.link_state && _global.state.program_enable;) {

		cs::GrabSinkFrame(ctx.fin_h, ctx.frame, &status);
		high_resolution_clock::time_point t = high_resolution_clock::now();
		float fps = 1.f / duration<float>(t - tp).count();
		tp = t;
		verbosity = _global.nt.ovl_verbosity.Get();
		if(verbosity <= 0) {
			cs::PutSourceFrame(ctx.fout_h, ctx.frame, &status);		// keep latency as short as possible
		}
		switch(ctx.procm) {
			case 0:
			default:{	// none
				if(verbosity > 0) {
					cv::putText(
						ctx.frame, std::to_string(fps),
						cv::Point(5, 20), cv::FONT_HERSHEY_DUPLEX,
						0.65, cv::Scalar(0, 255, 0), 1, cv::LINE_AA
					);
				}
				cs::PutSourceFrame(ctx.fout_h, ctx.frame, &status);
			}
			case 1: {	// apriltag
				if(ctx.vproc_buffer == nullptr) {
					ctx.vproc_buffer = new CThread::ProcBuff{};
				}
				ctx.vproc_buffer->corners.clear();
				ctx.vproc_buffer->ids.clear();
				// resize
				_ap_detect_aruco(
					ctx.frame, ctx.vproc_buffer->corners, ctx.vproc_buffer->ids);
				if(verbosity > 2) {

				} else if(verbosity > 1) {

				} else if(verbosity > 0) {
					cv::putText(
						ctx.frame, std::to_string(fps),
						cv::Point(5, 20), cv::FONT_HERSHEY_DUPLEX,
						0.65, cv::Scalar(0, 255, 0), 1, cv::LINE_AA
					);
				} else {
					goto skip_resend_frame;
				}
				cs::PutSourceFrame(ctx.fout_h, ctx.frame, &status);

				skip_resend_frame:
				// clear tvecs, rvecs?
				_ap_estimate_aruco(
					ctx.vproc_buffer->corners, ctx.vproc_buffer->ids,
					ctx.camera_matrix, ctx.dist_matrix,
					ctx.vproc_buffer->tvecs, ctx.vproc_buffer->rvecs);
				if(ctx.robot_camera_pose != nullptr) {
					// add the poses? :(
					_global.aprilpose.robot_queue_rw.lock();
					toPose3d(_global.aprilpose.robot_est_queue, ctx.vproc_buffer->tvecs, ctx.vproc_buffer->rvecs);
					_global.aprilpose.robot_queue_rw.unlock();
				}
#if SEND_EXTRA_POSE_ESTIMATIONS > 0
				else {
					_global.aprilpose.raw_queue_rw.lock();
					toPose3d(_global.aprilpose.raw_est_queue, ctx.vproc_buffer->tvecs, ctx.vproc_buffer->rvecs);
					_global.aprilpose.raw_queue_rw.lock();
				}
#endif
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



frc::Pose3d toPose3d(const CThread::ProcBuff::PVec& tvec, const CThread::ProcBuff::PVec& rvec) {
	cv::Mat_<float> R, t{3, 1, tvec.data()};
	cv::Rodrigues(rvec, R);
	R = R.t();
	t = -R * t;

	frc::Vectord<3> rv{ rvec[2], -rvec[0], rvec[1] };

	return Pose3d(
		units::inch_t{ +t.at<float>(2, 0) },
		units::inch_t{ -t.at<float>(0, 0) },
		units::inch_t{ -t.at<float>(1, 0) },
		frc::Rotation3d{ rv, units::radian_t{ rv.norm() } }
	);
}
std::span<frc::Pose3d> toPose3d(
	std::vector<frc::Pose3d>& poses,
	const std::vector<CThread::ProcBuff::PVec>& tvecs,
	const std::vector<CThread::ProcBuff::PVec>& rvecs
) {
	size_t beg = poses.size();
	for(size_t i = 0; i < tvecs.size(); i++) {
		poses.emplace_back(toPose3d(tvecs[i], rvecs[i]));
	}
	return std::span<frc::Pose3d>{ poses.begin() + beg, poses.end() };
}

void _ap_detect_aruco(
	const cv::Mat& frame,
	std::vector<std::vector<cv::Point2f>>& corners,
	std::vector<int32_t>& ids
) {
	cv::aruco::detectMarkers(
		frame, _global.aprilpose.field->dictionary,
		corners, ids, _global.aprilpose.params
	);
}
void _ap_detect_wpi(
	const cv::Mat& frame,
	frc::AprilTagDetector::Results& results
) {
	results = _global.aprilpose.wpi_detector.Detect(frame.cols, frame.rows, frame.data);
}

int _ap_estimate_aruco(
	cv::InputArrayOfArrays& corners, cv::InputArray& ids,
	cv::InputArray cmatx, cv::InputArray cdist,
	std::vector<CThread::ProcBuff::PVec>& tvecs, std::vector<CThread::ProcBuff::PVec>& rvecs
) {
	CV_Assert(corners.total() == ids.total());	
	CV_Assert(_global.aprilpose.field->getIds().size() == _global.aprilpose.field->getObjPoints().size());

	size_t ndetected = ids.total();
	std::vector<Point3f> _obj_points, _img_points;
	_obj_points.reserve(ndetected);
	_img_points.reserve(ndetected);

	for(size_t i = 0; i < ndetected; i++) {
		int id = ids.getMat().ptr<int>(0)[i];
		for(size_t j = 0; j < _global.aprilpose.field->getIds().size(); j++) {
			if(id == _global.aprilpose.field->getIds()[j]) {
				for(int p = 0; p < 4; p++) {
					_obj_points.push_back(_global.aprilpose.field->getObjPoints()[j][p]);
					_img_points.push_back(corners.getMat(i).ptr<Point2f>(0)[p]);
				}
			}
		}
	}

	if(obj_points.total() == 0) { return 0; }
	CV_Assert(img_points.total() == obj_points.total());
	
	if(img_points.total() == 4) {
		cv::solvePnPGeneric(
			_obj_points, _img_points,
			cmatx, cdist, rvecs, tvecs,
			false, cv::SOLVEPNP_IPPE_SQUARE
		);
	} else {
		cv::solvePnPGeneric(
			_obj_points, _img_points,
			cmatx, cdist, rvecs, tvecs,
			false, cv::SOLVEPNP_ITERATIVE
		);
	}
	return tvecs.size();

}


void _ap_update() {
	if(_global.nt.april_mode.Get() != 0) {	// if should be active
		if(!_global.aprilpose.queue.joinable()) {
			_global.aprilpose.link_state = true;
			_global.aprilpose.queue = std::thread(_ap_worker);
		}
	} else if(_global.aprilpose.queue.joinable()) {
		_global.aprilpose.link_state = false;
		_global.aprilpose.queue.join();
	}
}
void _ap_worker() {

	for(;_global.aprilpose.link_state && _global.state.program_enable) {
		_global.aprilpose.robot_queue_rw.lock();
		// handle poses
		_global.aprilpose.robot_est_queue.clear();
		_global.aprilpose.robot_queue_rw.unlock();
#if SEND_EXTRA_POSE_ESTIMATIONS > 0
		_global.aprilpose.raw_queue_rw.lock();
		// handle
		_global.aprilpose.raw_est_queue.clear();
		_global.aprilpose.raw_queue_rw.unlock();
#endif
	}

}


// void _apriltag() {

// 	int status = 0;
// 	CS_Sink cv_in = cs::CreateCvSink("apriltag_cv_in", &status);
// 	CS_Source cv_out = cs::CreateCvSource("apriltag_cv_out", _global.camera_vmode, &status);
// 	cs::SetSinkSource(cv_in, _global.camera_src_handle, &status);
// 	cs::SetSinkSource(_global.h_stream, cv_out, &status);

// 	cv::Mat frame;
// 	high_resolution_clock::time_point tp = high_resolution_clock::now();

// 	for(;_global.s_program_enable;) {

// 		uint64_t dt = cs::GrabSinkFrame(cv_in, frame, &status);
// 		high_resolution_clock::time_point t = high_resolution_clock::now();
// 		float fps = 1.f / duration<float>(t - tp).count();
// 		tp = t;
// 		cv::putText(
// 			frame, std::to_string(fps),
// 			cv::Point(5, 20), cv::FONT_HERSHEY_DUPLEX,
// 			0.65, cv::Scalar(0, 255, 0), 1, cv::LINE_AA
// 		);
// 		cs::PutSourceFrame(cv_out, frame, &status);

// 		std::this_thread::sleep_for(20ms);
// 	}

// 	cs::ReleaseSink(cv_in, &status);
// 	cs::ReleaseSource(cv_out, &status);

// }
