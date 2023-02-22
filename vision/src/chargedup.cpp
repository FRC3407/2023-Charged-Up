#include <iostream>
#include <csignal>
#include <thread>
#include <atomic>
#include <chrono>
#include <vector>
#include <array>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <fmt/format.h>

#include <cscore_cv.h>
#include <cscore_cpp.h>
// #include <wpi/sendable/Sendable.h>

#include <cpp-tools/src/sighandle.h>


using namespace std::chrono_literals;

bool init();

void _apriltag();
void _retroreflective();
void _gamepieces();

struct {
	std::atomic<bool> s_program_enable{true};
	std::chrono::system_clock::time_point start_time;

	cs::VideoMode camera_vmode = {
		cs::VideoMode::kMJPEG, 640, 480, 30};
	CS_Source camera_src_handle;
	CS_Sink output_stream;

} _global;

int main(int argc, char** argv) {

	int status = 0;
	_global.start_time = std::chrono::system_clock::now();
	if(!init()) {
		std::cout << "Unable to find compatible USB camera. Exitting..." << std::endl;
		return EXIT_FAILURE;
	}
	// initialize nt

	cs::SetCameraExposureManual(
		_global.camera_src_handle, 20, &status);
	// cs::SetSinkSource(
	// 	_global.output_stream,
	// 	_global.camera_src_handle,
	// 	&status);

	std::thread		// start the processing threads
		april_runner{_apriltag},
		retro_runner{_retroreflective};

	// setup exit sigaction
	{
		struct sigaction _action;
		sigemptyset(&(_action.sa_mask));
		_action.sa_flags = SA_SIGINFO;
		_action.sa_sigaction = [&_global](int, siginfo_t*, void*){
			_global.s_program_enable = false;
		};
		sigaction((unsigned int)SigHandle::Sigs::INT, &_action, nullptr);
		sigaction((unsigned int)SigHandle::Sigs::QUIT, &_action, nullptr);
		sigaction((unsigned int)SigHandle::Sigs::ILL, &_action, nullptr);
		sigaction((unsigned int)SigHandle::Sigs::SEGV, &_action, nullptr);
	}

	for(;_global.s_program_enable;) {

		// handle nt updates for cameras, etc
		// handle exit conditions

		std::cout << "Running mainloop" << std::endl;

		std::this_thread::sleep_for(1s);
	}

	std::cout << "\nExitting..." << std::endl;

	if(april_runner.joinable()) april_runner.join();
	if(retro_runner.joinable()) retro_runner.join();

	// deinit
	cs::ReleaseSource(_global.camera_src_handle, &status);
	cs::ReleaseSink(_global.output_stream, &status);

	std::cout << "Runtime: " <<
		std::chrono::duration<double>(
			std::chrono::system_clock::now() - _global.start_time).count() << 's' << std::endl;

	return EXIT_SUCCESS;

}



bool init() {
	int status = 0;
	std::vector<cs::UsbCameraInfo> _cameras =
		cs::EnumerateUsbCameras(&status);
	_global.output_stream = cs::CreateMjpegServer("Test Stream", "", 1181, &status);
	cs::SetProperty(
		cs::GetSinkProperty(_global.output_stream, "width", &status),
		_global.camera_vmode.width, &status);
	cs::SetProperty(
		cs::GetSinkProperty(_global.output_stream, "height", &status),
		_global.camera_vmode.height, &status);
	cs::SetProperty(
		cs::GetSinkProperty(_global.output_stream, "fps", &status),
		_global.camera_vmode.fps, &status);
	for(unsigned int i = 0; i < _cameras.size(); i++) {
		if(strncmp(_cameras[i].name.c_str(), "bcm", 3) != 0) {
			std::cout << "Camera Detected: " << _cameras[i].name << std::endl;

			_global.camera_src_handle = cs::CreateUsbCameraPath(
				"Test Camera", _cameras[i].path, &status);
			cs::SetSourceConnectionStrategy(
				_global.camera_src_handle,
				CS_CONNECTION_KEEP_OPEN, &status);
			cs::SetSourceVideoMode(
				_global.camera_src_handle,
				_global.camera_vmode, &status);

			// std::vector<cs::VideoMode> _videomodes =
			// 	cs::EnumerateSourceVideoModes(_global.camera_src_handle, &status);
			// std::cout << "Camera Properties:" << std::endl;
			// for(unsigned int v = 0; v < _videomodes.size(); v++) {
			// 	std::cout << "Mode " << v << ":\n\tFormat: " <<
			// 		_videomodes[v].pixelFormat << "\n\tFSize: [" <<
			// 		_videomodes[v].width << "x" << _videomodes[v].height << "]\n\tFPS: " <<
			// 		_videomodes[v].fps << std::endl;
			// }
			return true;
		}
	}
	return false;
}

void _apriltag() {

	int status = 0;
	CS_Sink cv_in = cs::CreateCvSink("apriltag_cv_in", &status);
	CS_Source cv_out = cs::CreateCvSource("apriltag_cv_out", _global.camera_vmode, &status);
	cs::SetSinkSource(cv_in, _global.camera_src_handle, &status);
	cs::SetSinkSource(_global.output_stream, cv_out, &status);

	cv::Mat frame;
	std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();

	for(;_global.s_program_enable;) {

		uint64_t dt = cs::GrabSinkFrame(cv_in, frame, &status);
		std::chrono::high_resolution_clock::time_point t = std::chrono::high_resolution_clock::now();
		float fps = 1.f / std::chrono::duration<float>(t - tp).count();
		tp = t;
		cv::putText(
			frame, std::to_string(fps),
			cv::Point(5, 20), cv::FONT_HERSHEY_DUPLEX,
			0.65, cv::Scalar(0, 255, 0), 1, cv::LINE_AA
		);
		cs::PutSourceFrame(cv_out, frame, &status);

		std::this_thread::sleep_for(20ms);
	}

	cs::ReleaseSink(cv_in, &status);
	cs::ReleaseSource(cv_out, &status);

}
void _retroreflective() {

	int status = 0;
	CS_Sink cv_in = cs::CreateCvSink("retroref_cv_in", &status);
	CS_Source cv_out = cs::CreateCvSource("retroref_cv_out", _global.camera_vmode, &status);
	cs::SetSinkSource(cv_in, _global.camera_src_handle, &status);

	for(;_global.s_program_enable;) {

		std::this_thread::sleep_for(20ms);
	}

	cs::ReleaseSink(cv_in, &status);
	cs::ReleaseSource(cv_out, &status);

}