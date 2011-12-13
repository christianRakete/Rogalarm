#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/Fbo.h"
#include "cinder/Surface.h"
#include "cinder/Utilities.h"

#include "Kinect.h"

#include "cinder/params/Params.h"

#include "Resources.h"
#include "Constants.h"

#include "CinderOpenCV.h"
#include "OscSender.h"
#include "SimpleGUI.h"

using namespace ci;
using namespace ci::app;
using namespace std;

using namespace mowa::sgui;

class RogalarmApp : public AppBasic {
  public:
	void prepareSettings( Settings *settings );
	void setup();
	void mouseDown( MouseEvent event );
	void keyDown( KeyEvent event );
	void resize( ResizeEvent event );
	void update();
	void draw();
    bool openSaveConfigClick( MouseEvent event );
    void lookingForUser();

	gl::Texture		mFrameTexture, mInfoTexture;
	gl::Texture		mColorTexture, mDepthTexture, mContourTexture;
	gl::Texture		mSplash, mKonnect;
	cv::Mat			mBackground;
	
	float	mUserPos;
	float	mStopedTime, mDebugTime;
	float	mThreshold, mBlobMin, mBlobMax;
	
	int		mDebugTimeThreshold;
	int		mReflectionTop, mReflectionBottom;
	int		mSequencePos;
	
	bool	mFullscreen, mUser, mDebug, mNoUserMessage;
	bool	mKinectConected;
    
    string mConfigPath;
	
	osc::Sender mSender;
	std::string mHost;
	int mPort;
	
	SimpleGUI* mGui;
    
	Kinect mKinect;
	
	Surface mDepthSurface, mDepthBackGroundSurface;
	Vec3f mTargetPosition;



};

void RogalarmApp::prepareSettings( Settings *settings )
{
	settings->setFrameRate( kFrameRate );
}

void RogalarmApp::setup()
{
	mFullscreen = false;
	mUser		= false;
	mNoUserMessage    = false;
	setFullScreen(mFullscreen);
	mUserPos = 0;
	mDebug = false;	
	mKonnect = loadImage(loadResource("konnect-kinect.jpg") );
	
	try {
		mKinect = Kinect( Kinect::Device() );
		mKinectConected = true;
		
	} catch( ... ){
		mKinectConected = false;
	}
    
    
    mConfigPath = getResourcePath().c_str();
    mConfigPath += "settings.sgui.txt";
   
	
    
	mStopedTime = getElapsedSeconds();
	
	
	mTargetPosition = Vec3f::zero();
	
	mContourTexture = gl::Texture(getWindowWidth(), getWindowHeight());
	mDepthTexture = gl::Texture(getWindowWidth(), getWindowHeight());
	
	mGui = new SimpleGUI(this);
	mGui->lightColor = ColorA(1, 1, 0, 1);
	
	mGui->addLabel("CONTROLS");
	mGui->addParam("Depth Threshold", &mThreshold, 0, 255, 70);
	mGui->addParam("Min Contour Area", &mBlobMin, 10, 100, 30);
	mGui->addParam("Max Contour Area", &mBlobMax, 100, 500, 200);
	mGui->addSeparator();
	mGui->addParam("Reflection Top", &mReflectionTop, 0, 480, 0);
	mGui->addParam("Reflection Bottom", &mReflectionBottom, 0, 480, 0);
	mGui->addSeparator();
	mGui->addLabel("OPTIONS");
	mGui->addParam("Fullscreen (f)", &mFullscreen, false);
	mGui->addButton("Save Configuration")->registerClick(this, &RogalarmApp::openSaveConfigClick);
	mGui->addColumn(142, 7);
	mGui->addLabel("Contour Image");
	mGui->addParam("Contour Texture", &mContourTexture); 
	mGui->addLabel("Depth Image");
	mGui->addParam("Depth Texture", &mDepthTexture);
	
	mGui->load(mConfigPath);
	
	mHost = "10.0.1.137";
	mPort = 7110;
	mSender.setup(mHost, mPort);



}

void RogalarmApp::mouseDown( MouseEvent event )
{
}

bool RogalarmApp::openSaveConfigClick( MouseEvent event ) {
	mGui->save(mConfigPath);
	return false;
}

void RogalarmApp::resize( ResizeEvent event )
{
//	mFbo = gl::Fbo( getWindowWidth(), getWindowHeight() );
}


void RogalarmApp::update()
{
    setFullScreen(mFullscreen);
	
	
	if (mKinectConected) {
		lookingForUser();
		
	}
	
	osc::Message message;
	message.addFloatArg(mUserPos);
	if (mUser) {
		message.setAddress("/user/1");
	} else {
		message.setAddress("/nouser");
	}
	message.setRemoteEndpoint(mHost, mPort);
	mSender.sendMessage(message);
	
	if (!mKinectConected) {
		try {
			mKinect = Kinect( Kinect::Device() );
			mKinectConected = true;
			
		} catch( ... ){
			mKinectConected = false;
		}
	}


}

void RogalarmApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) );
	gl::enableAlphaBlending();
	
	if (mKinectConected) {
        
		//Rectf centeredRect = Rectf( mDepthTexture.getBounds() ).getCenteredFit( getWindowBounds(), true );
		//gl::draw( mDepthTexture, centeredRect  );
		
		
		//gl::drawLine(Vec2f(mUserPos, 0), Vec2f(mUserPos, 640));
		gl::color (Color8u(255 ,255, 0));
		gl::drawSolidRect (Rectf(mUserPos-2, 0, mUserPos+2, 640) ,false);
        
		mGui->draw();
        
	}else {
		Rectf centeredRect2 = Rectf( mKonnect.getBounds() ).getCenteredFit( getWindowBounds(), true );
		gl::draw( mKonnect, centeredRect2  );
	}

}

void RogalarmApp::lookingForUser() {
	if( mKinect.checkNewDepthFrame() ){
		
		
		ImageSourceRef depthImage = mKinect.getDepthImage();
		
		// make a texture to display
		mDepthTexture = depthImage;
		// make a surface for opencv
		mDepthSurface = depthImage;
		
		if(mDepthSurface){
			
			// once the surface is avalable pass it to opencv
			// had trouble here with bit depth. surface comes in full color, needed to crush it down
			cv::Mat tmp( toOcv( Channel8u( mDepthSurface )  ) ), input, blurred, thresholded, thresholded2, output;
			
			if (mReflectionBottom > 0) {
				cv::Scalar black( 0, 0, 0 );
				cv::Point p1 = cv::Point(0,480 - mReflectionBottom-1);
				cv::Point p2 = cv::Point(640, 480);
				cv::rectangle(tmp, p1, p2, black, -1, 8, 0);
			}
			
			if (mReflectionTop > 0) {
				cv::Scalar black( 0, 0, 0 );
				cv::Point p1 = cv::Point(0, 0);
				cv::Point p2 = cv::Point(640, mReflectionTop);
				cv::rectangle(tmp, p1, p2, black, -1, 8, 0);
			}
			
			//tmp.copyTo(input, mBackground);
			
			cv::blur(tmp, blurred, cv::Size(10,10));
			
			// make two thresholded images one to display and one
			// to pass to find contours since its process alters the image
			cv::threshold( blurred, thresholded, mThreshold, 255, CV_THRESH_BINARY);
			cv::threshold( blurred, thresholded2, mThreshold, 255, CV_THRESH_BINARY);
			
			// 2d vector to store the found contours
			vector<vector<cv::Point> > contours;
			// find em
			cv::findContours(thresholded, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
			
			// convert theshold image to color for output
			// so we can draw blobs on it
			cv::cvtColor( thresholded2, output, CV_GRAY2RGB );
			cv::Scalar color( 0, 255, 255 );
			mUser = false;
			
			// loop the stored contours
			for (vector<vector<cv::Point> >::iterator it=contours.begin() ; it < contours.end(); it++ ){
				
				// center abd radius for current blob
				cv::Point2f center;
				float radius;
				// convert the cuntour point to a matrix 
				vector<cv::Point> pts = *it;
				cv::Mat pointsMatrix = cv::Mat(pts);
				// pass to min enclosing circle to make the blob 
				cv::minEnclosingCircle(pointsMatrix, center, radius);
				
				
				if (radius > mBlobMin && radius < mBlobMax) {
					mUserPos = 640 - center.x;
					mUser = true;
					if (mNoUserMessage == false) {
						mNoUserMessage = true;
					}
					
					cv::circle(output, center, radius, color, 3);
					
					mStopedTime = getElapsedSeconds();
					
					
					osc::Message message;
					message.addFloatArg(mUserPos);
					message.setAddress("/user/1");
					message.setRemoteEndpoint(mHost, mPort);
					mSender.sendMessage(message);
					
                    
				} else if (mNoUserMessage) {
					osc::Message message;
					message.addFloatArg(mUserPos);
					message.setAddress("/nouser");
					message.setRemoteEndpoint(mHost, mPort);
					mSender.sendMessage(message);
					
					mNoUserMessage = false;
					
				} 					
			}	
			
			cv::Scalar yellow( 0, 255, 255 );
			
			if (mReflectionBottom > 0) {
				
				cv::Point p1 = cv::Point(0, 480 - mReflectionBottom-1);
				cv::Point p2 = cv::Point(640, 480);
				cv::rectangle(output, p1, p2, yellow, -1, 8, 0);
			}
			
			if (mReflectionTop > 0) {
				cv::Scalar black( 0, 0, 0 );
				cv::Point p1 = cv::Point(0, 0);
				cv::Point p2 = cv::Point(640, mReflectionTop);
				cv::rectangle(output, p1, p2, yellow, -1, 8, 0);
			}
			
			mContourTexture = gl::Texture( fromOcv( output ) );
		}
	}
    
	if( mKinect.checkNewVideoFrame() )
		mColorTexture = mKinect.getVideoImage();
    
}

void RogalarmApp::keyDown( KeyEvent event )
{
	if( event.getChar() == 'f' ){
		mFullscreen = !mFullscreen; 
		setFullScreen(mFullscreen);
	} else if( event.getChar() == 'p' ){
		cout << "User Position = " << mUserPos << endl;
	} else if( event.getChar() == 'd' ){
		mGui->dump();
	} 
	
}


CINDER_APP_BASIC( RogalarmApp, RendererGl(0) )
