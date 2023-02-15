#include "cpp-tools/src/resources.h"
#include "cpp-tools/src/sighandle.h"
#include "cpp-tools/src/timing.h"

#include <core/visionserver2.h>
#include <core/visioncamera.h>
#define APRILPOSE_DEBUG
#include <core/aprilpose.h>
#include <core/config.h>
#include <core/calib.h>

#include <vector>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/geometry/Pose2d.h>
#include <wpi/sendable/Sendable.h>

#include "field.h"


static const inline CalibList
	calibrations{
		{
			{ "lifecam_hd3000", {
				{ cv::Size{640, 480}, {
					cv::Mat1f{{3, 3},{
						673.6653136395231, 0, 339.861572657799,
						0, 666.1104961259615, 244.21065776461745,
						0, 0, 1}},
					cv::Mat1f{{1, 5},{
						0.04009256446529976, -0.4529245799337021,
						-0.001655316303789686, -0.00019284071985319236,
						0.5736326357832554}}
				} }
			} }
		}
	}
;

class AprilPoseExt : public AprilPose_<AprilPoseExt> {
public:
	inline AprilPoseExt(
		const cv::Ptr<cv::aruco::Board> f,
		cv::Ptr<cv::aruco::DetectorParameters> p = cv::aruco::DetectorParameters::create()
	) : AprilPose_<AprilPoseExt>(f, p) {
		frc::SmartDashboard::PutData("Robot Location", &this->position);
	}

	virtual void process(cv::Mat& io_frame) override {
		this->_proc(io_frame);
		
		cv::Mat_<float> R, t, T;
		cv::Rodrigues(this->rvec, R);		// get rotation matrix (3x3)
		R = R.t();							// invert rotation mat
		t = cv::Mat_<float>(3, 1, this->tvec.data());
		// std::cout << R << std::endl;
		// std::cout << t << std::endl;
		t = -R * t;						// invert tvec
		T = cv::Mat::eye(4, 4, R.type());	// generate 4x4 blank transformation mat
		T(cv::Range(0, 3), cv::Range(0, 3)) = R * 1;	// copy R into the top-leftmost 3x3 locations
		T(cv::Range(0, 3), cv::Range(3, 4)) = t * 1; 	// copy t into rightmost top 3 locations

		this->getTable()->PutNumber("X", t.at<float>(0, 0));
		this->getTable()->PutNumber("Y", t.at<float>(0, 1));
		this->getTable()->PutNumber("Z", t.at<float>(0, 2));
		this->getTable()->PutNumber("RX", this->rvec[0] / CV_PI * 180.f);
		this->getTable()->PutNumber("RY", this->rvec[1] / CV_PI * 180.f);
		this->getTable()->PutNumber("RZ", this->rvec[2] / CV_PI * 180.f);
		frc::Pose2d pose = frc::Pose2d(
			units::inch_t(t.at<float>(0, 0)),
			units::inch_t(t.at<float>(0, 1)),
			frc::Rotation2d(units::radian_t(this->rvec[1]))
		);
		this->position.SetRobotPose(pose);
	}

private:
	frc::Field2d position;


};


StopWatch runtime("Runtime", &std::cout, 0);
void on_exit() { runtime.end(); }

int main(int argc, char** argv) {
	runtime.setStart();
	SigHandle::get();
	atexit(on_exit);

	std::vector<VisionCamera> cameras;

	if(argc > 1 && initNT(argv[1]) && createCameras(cameras, calibrations, argv[1])) {}
	else if(initNT() && createCameras(cameras, calibrations)) {}
	else { return EXIT_FAILURE; }

	cv::Ptr<cv::aruco::DetectorParameters> aparams = cv::aruco::DetectorParameters::create();
	AprilPoseExt ap{FIELD_2023, aparams};

	using namespace vs2;
	VisionServer::Init();
	VisionServer::addCameras(std::move(cameras));
	VisionServer::addStreams(1);
	VisionServer::addPipeline(&ap);
	VisionServer::compensate();
	VisionServer::run(60.f);
	atexit(VisionServer::stopExit);

}