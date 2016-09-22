#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <itkImage.h>
#include <itkCurvatureFlowImageFilter.h>
#include <itkOpenCVImageBridge.h>

#include "itkImageRegistrationMethodv4.h"
#include "itkCenteredRigid2DTransform.h"
#include "itkCenteredTransformInitializer.h"
#include "itkMattesMutualInformationImageToImageMetricv4.h"
#include "itkRegularStepGradientDescentOptimizerv4.h"

#include "itkResampleImageFilter.h"
#include "itkCastImageFilter.h"

#include "itkCommand.h"

#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include "std_msgs/Int32.h"
#include "std_msgs/UInt64.h"
#include "std_msgs/Float32.h"

#include "entropy/MI.h"

using namespace std;
using namespace cv;

class CommandIterationUpdate : public itk::Command
{
public:
  ros::Publisher* iters_pub;
  ros::Publisher* mi_pub;

  typedef  CommandIterationUpdate   Self;
  typedef  itk::Command             Superclass;
  typedef itk::SmartPointer<Self>   Pointer;
  itkNewMacro( Self );

protected:
  CommandIterationUpdate() {};

public:
  typedef itk::RegularStepGradientDescentOptimizerv4<double>  OptimizerType;
  typedef   const OptimizerType *                             OptimizerPointer;

  void Execute(itk::Object *caller, const itk::EventObject & event) ITK_OVERRIDE
    {
    Execute( (const itk::Object *)caller, event);
    }

  void Execute(const itk::Object * object, const itk::EventObject & event) ITK_OVERRIDE
    {
    OptimizerPointer optimizer = static_cast< OptimizerPointer >( object );
    if( ! itk::IterationEvent().CheckEvent( &event ) )
      {
      return;
      }

      //unsigned int numberOfIterations = optimizer->GetCurrentIteration();
      std_msgs::UInt64 iter;
      // std_msgs::Float32 mi;
      entropy::MI mi;

      iter.data = optimizer->GetCurrentIteration();//numberOfIterations;
      // mi.data = optimizer->GetValue();
      mi.header.stamp = ros::Time::now();
      mi.val = optimizer->GetValue();

      if(iters_pub != nullptr)
      	iters_pub->publish(iter);

      if(mi_pub != nullptr)
      	mi_pub->publish(mi);
    // std::cout << optimizer->GetCurrentIteration() << "   ";
    // std::cout << optimizer->GetValue() << "   ";
    // std::cout << optimizer->GetCurrentPosition() << std::endl;
    }
  void SetMIPublisher(ros::Publisher& pub)
  {
  	mi_pub = &pub;
  };
  void SetItersPublisher(ros::Publisher& pub)
  {
  	iters_pub = &pub;
  };
};

class MIEstimator
{
private:
	ros::Subscriber sub;
	ros::Publisher mi_pub;
	ros::Publisher iters_pub;

	cv::Mat prevImage;
	bool isFirstFrame;

public:
	MIEstimator(int argc, char *argv[]);
	int GetSolution(cv::Mat& prevImage, cv::Mat& nextImage);
	void GrabImage(const sensor_msgs::ImageConstPtr& msg);
};

int MIEstimator::GetSolution(cv::Mat& prevImage, cv::Mat& nextImage)
{
	cout << "GetSolution..." << endl;

	imshow("prev", prevImage);
	imshow("next", nextImage);
	waitKey(1);

	const unsigned int Dimension = 2;

	typedef float PixelType;
	typedef itk::Image< PixelType, Dimension >  ImageType;
	typedef itk::CenteredRigid2DTransform< double >  TransformType;
	typedef itk::RegularStepGradientDescentOptimizerv4<double> OptimizerType;
	typedef itk::ImageRegistrationMethodv4< ImageType, ImageType, TransformType > RegistrationType;
	typedef itk::MattesMutualInformationImageToImageMetricv4< ImageType, ImageType >  MetricType;
	typedef itk::OpenCVImageBridge BridgeType;

	ImageType::Pointer fixImage = BridgeType::CVMatToITKImage< ImageType >( prevImage );
	ImageType::Pointer movImage = BridgeType::CVMatToITKImage< ImageType >( nextImage );

	TransformType::Pointer      transform     = TransformType::New();
	MetricType::Pointer         metric        = MetricType::New();
	OptimizerType::Pointer      optimizer     = OptimizerType::New();
	RegistrationType::Pointer   registration  = RegistrationType::New();

	registration->SetOptimizer( optimizer );
	registration->SetMetric( metric );

	metric->SetNumberOfHistogramBins( 20 );//20
	double samplingPercentage = 0.20;
	registration->SetMetricSamplingPercentage( samplingPercentage );
	RegistrationType::MetricSamplingStrategyType  samplingStrategy = RegistrationType::RANDOM;
	registration->SetMetricSamplingStrategy( samplingStrategy );

	registration->SetFixedImage( fixImage );
	registration->SetMovingImage( movImage );

	typedef itk::CenteredTransformInitializer< TransformType, ImageType, ImageType > TransformInitializerType;

	TransformInitializerType::Pointer initializer = TransformInitializerType::New();
	initializer->SetTransform(   transform );

	initializer->SetFixedImage( fixImage );
	initializer->SetMovingImage( movImage );
	initializer->GeometryOn();
	initializer->InitializeTransform();
	transform->SetAngle( 0.0 );
	registration->SetInitialTransform( transform );
	registration->InPlaceOn();
	typedef OptimizerType::ScalesType       OptimizerScalesType;
	OptimizerScalesType optimizerScales( transform->GetNumberOfParameters() );
	const double translationScale = 1.0 / 128.0;
	const double centerScale      = 1000.0; // prevents it from moving
	                                          // during the optimization
	optimizerScales[0] = 1.0;
	optimizerScales[1] = centerScale;
	optimizerScales[2] = centerScale;
	optimizerScales[3] = translationScale;
	optimizerScales[4] = translationScale;
	optimizer->SetScales( optimizerScales );
	optimizer->SetLearningRate( 0.5 );
	optimizer->SetMinimumStepLength( 0.0001 );//0.0001 
	optimizer->SetNumberOfIterations( 400 ); //400

	// One level registration process without shrinking and smoothing.
	//
	const unsigned int numberOfLevels = 1;
	RegistrationType::ShrinkFactorsArrayType shrinkFactorsPerLevel;
	shrinkFactorsPerLevel.SetSize( 1 );
	shrinkFactorsPerLevel[0] = 1;
	RegistrationType::SmoothingSigmasArrayType smoothingSigmasPerLevel;
	smoothingSigmasPerLevel.SetSize( 1 );
	smoothingSigmasPerLevel[0] = 0;
	registration->SetNumberOfLevels ( numberOfLevels );
	registration->SetSmoothingSigmasPerLevel( smoothingSigmasPerLevel );
	registration->SetShrinkFactorsPerLevel( shrinkFactorsPerLevel );

	// Create the Command observer and register it with the optimizer.
	//
	CommandIterationUpdate::Pointer observer = CommandIterationUpdate::New();
	
	observer->SetItersPublisher(iters_pub);
	observer->SetMIPublisher(mi_pub);

	optimizer->AddObserver( itk::IterationEvent(), observer );

	try
	{
		registration->Update();
		std::cout << "Optimizer stop condition = "
		          << registration->GetOptimizer()->GetStopConditionDescription()
		          << std::endl;
	}
	catch( itk::ExceptionObject & err )
	{
		std::cout << "ExceptionObject caught !" << std::endl;
		std::cout << err << std::endl;
		return EXIT_FAILURE;
	}

	typedef TransformType::ParametersType ParametersType;
	ParametersType finalParameters = transform->GetParameters();

	const double finalAngle           = finalParameters[0];
	const double finalRotationCenterX = finalParameters[1];
	const double finalRotationCenterY = finalParameters[2];
	const double finalTranslationX    = finalParameters[3];
	const double finalTranslationY    = finalParameters[4];

	unsigned int numberOfIterations = optimizer->GetCurrentIteration();

	double bestValue = optimizer->GetValue();

	// Print out results
	//

	const double finalAngleInDegrees = finalAngle * 180 / vnl_math::pi;

	std::cout << "Result = " << std::endl;
	std::cout << " Angle (radians) " << finalAngle  << std::endl;
	std::cout << " Angle (degrees) " << finalAngleInDegrees  << std::endl;
	std::cout << " Center X      = " << finalRotationCenterX  << std::endl;
	std::cout << " Center Y      = " << finalRotationCenterY  << std::endl;
	std::cout << " Translation X = " << finalTranslationX  << std::endl;
	std::cout << " Translation Y = " << finalTranslationY  << std::endl;
	std::cout << " Iterations    = " << numberOfIterations << std::endl;
	std::cout << " Metric value  = " << bestValue          << std::endl;	
}


void MIEstimator::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
	cv::Mat nextImage;

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    ROS_ASSERT(cv_ptr->image.channels()==3 || cv_ptr->image.channels()==1);

    if(cv_ptr->image.channels()==3)
    {
        cvtColor(cv_ptr->image, nextImage, CV_BGR2GRAY);
    }
    else if(cv_ptr->image.channels()==1)
    {
        cv_ptr->image.copyTo(nextImage);
    }

    // cv_ptr->image.copyTo(prevImage);
	// imshow("prev", prevImage);
	// imshow("next", nextImage);

    if(isFirstFrame)
    {
    	isFirstFrame = false;
    	nextImage.copyTo(prevImage);
    }
    else
    {
    	// imshow("next", nextImage);
    	// imshow("prev", prevImage);
    	GetSolution(prevImage, nextImage);
    	nextImage.copyTo(prevImage);    	
    }

    //publish result
    std::cout << "GrabFrame"<< std::endl;
}

MIEstimator::MIEstimator(int argc, char *argv[])
{
	cout << "Starting node..." << endl;
	ros::NodeHandle nh("~");

	// sub = nh.subscribe("/image_raw", 1, &MIEstimator::GrabImage, this);
	sub = nh.subscribe("/camera/image_color", 1, &MIEstimator::GrabImage, this);
	iters_pub = nh.advertise<std_msgs::UInt64>("/iters", 1000);
	// mi_pub = nh.advertise<std_msgs::Float32>("/mi", 1000);
	mi_pub = nh.advertise<entropy::MI>("/mi", 1000);

	std::string filename = "itkmi_config.yml";
	std::string pathFilename = ros::package::getPath("itkmi")+"/param/"+filename;

	cout << "Loading config..." << endl;
	FileStorage fs( pathFilename, FileStorage::READ );
	if (!fs.isOpened())
	{
	    cerr << "Failed to open " << pathFilename << endl;
	    cerr << "Using default parameters." << endl;
	    // return false;
	}
	else
	{
		// fs["camera_matrix"] >> K;
		// fs["distortion_coefficients"] >> D;
		// fs["avg_reprojection_error"] >> totalAvgErr;
		cout << "Loaded." << endl;
	}
	fs.release();

	int framerate = 100;

	isFirstFrame = true;

    ros::Rate loop_rate(framerate);
	while(nh.ok())
	{
	  waitKey(1);
	  ros::spinOnce();
	  loop_rate.sleep();
	}	
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "itkmi");
	cout << "Running" << endl;
	MIEstimator* mi = new MIEstimator(argc, argv);
	cout << "Exiting" << endl;
	return 0;
}
