
#include <iostream>

using namespace xn;
using namespace std;

class OpenNI
{
public:
	OpenNI()
	{
		
	}
	~OpenNI() 
	{
		context.Release();
	}
	bool Initial() 
	{
		//init
		status = context.Init();
		if(CheckError("Context initial failed!")) {
			return false;
		}
		//context.SetGlobalMirror(true);//set mirror
		//generate image node
		status = image_generator.Create(context);
		if(CheckError("Create image generator error!")) {
			return false;
		}
		//generate depth image node
		status = depth_generator.Create(context);
		if(CheckError("Create depth generator error!")) {
			return false;
		}
		//Calibration
		status = depth_generator.GetAlternativeViewPointCap().SetViewPoint(image_generator);
		if(CheckError("Can't set the alternative view point on depth generator")) {
			return false;
		}

		return true;

	}

	bool Start() {
		status = context.StartGeneratingAll();
		if(CheckError("Start generating error!")) {
			return false;
		}
		return true;
	}

	bool UpdateData() {
		status = context.WaitNoneUpdateAll();
		if(CheckError("Update date error!")) {
			return false;
		}
		//Get data
		image_generator.GetMetaData(image_metadata);
		depth_generator.GetMetaData(depth_metadata);

		return true;
	}

public:
	DepthMetaData depth_metadata;
	ImageMetaData image_metadata;

private:
	//Check error. False means correct
	bool CheckError(const char* error) {
		if(status != XN_STATUS_OK ) {
			cerr << error << ": " << xnGetStatusString( status ) << endl;
			return true;
		}
		return false;
	}

private:
	XnStatus    status;
	Context     context;
	DepthGenerator  depth_generator;
	ImageGenerator  image_generator;
};