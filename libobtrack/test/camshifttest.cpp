#ifdef _CH_
#pragma package <opencv>
#endif

#define CV_NO_BACKWARD_COMPATIBILITY

#ifndef _EiC
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <ctype.h>
#include "obtrack.h"
#endif

cv::Mat frame;

int backproject_mode = 0;
int select_object = 0;
int track_object = 0;
int show_hist = 1;
CvPoint origin;
CvRect selection;
CvRect track_window;
CvBox2D track_box;
CvConnectedComp track_comp;
int hdims = 16;
float hranges_arr[] = {0,180};
float* hranges = hranges_arr;
int vmin = 10, vmax = 256, smin = 30;

void on_mouse( int event, int x, int y, int flags, void* param )
{    
    /*if( image.origin )
        y = image.height - y;*/

	if(event == CV_EVENT_LBUTTONUP)
		x = x;

    if( select_object )
    {
        selection.x = MIN(x,origin.x);
        selection.y = MIN(y,origin.y);
        selection.width = selection.x + CV_IABS(x - origin.x);
        selection.height = selection.y + CV_IABS(y - origin.y);

        selection.x = MAX( selection.x, 0 );
        selection.y = MAX( selection.y, 0 );
		selection.width = MIN( selection.width, frame.cols);
        selection.height = MIN( selection.height, frame.rows );
        selection.width -= selection.x;
        selection.height -= selection.y;
    }














    switch( event )
    {
    case CV_EVENT_LBUTTONDOWN:
        origin = cvPoint(x,y);
        selection = cvRect(x,y,0,0);
        select_object = 1;
        break;
    case CV_EVENT_LBUTTONUP:
        select_object = 0;
        if( selection.width > 0 && selection.height > 0 )
            track_object = -1;
        break;
    }
}


CvScalar hsv2rgb( float hue )
{
    int rgb[3], p, sector;
    static const int sector_data[][3]=
        {{0,2,1}, {1,2,0}, {1,0,2}, {2,0,1}, {2,1,0}, {0,1,2}};
    hue *= 0.033333333333333333333333333333333f;
    sector = cvFloor(hue);
    p = cvRound(255*(hue - sector));
    p ^= sector & 1 ? 255 : 0;

    rgb[sector_data[sector][0]] = 255;
    rgb[sector_data[sector][1]] = 0;
    rgb[sector_data[sector][2]] = p;

    return cvScalar(rgb[2], rgb[1], rgb[0],0);
}

int main( int argc, char** argv )
{
	cv::VideoCapture cap;
	bool capture = false;

    if( argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
        capture = cap.open(argc == 2 ? argv[1][0] - '0' : 0 );
    else if( argc == 2 )
        capture = cap.open( argv[1] );

    if( !capture )
    {
        fprintf(stderr,"Could not initialize capturing...\n");
        return -1;
    }

    printf( "Hot keys: \n"
        "\tESC - quit the program\n"
        "\tc - stop the tracking\n"
        "\tb - switch to/from backprojection view\n"
        "\th - show/hide object histogram\n"
        "To initialize tracking, select the object with mouse\n" );

    //cvNamedWindow( "Histogram", 1 );
    cvNamedWindow( "CamShiftDemo", 1 );
    cvSetMouseCallback( "CamShiftDemo", on_mouse, 0 );
    cvCreateTrackbar( "Vmin", "CamShiftDemo", &vmin, 256, 0 );
    cvCreateTrackbar( "Vmax", "CamShiftDemo", &vmax, 256, 0 );
    cvCreateTrackbar( "Smin", "CamShiftDemo", &smin, 256, 0 );

	obt::CamShiftTracker track;

    for(;;)
    {
        int i, bin_w, c;

		bool hasNextFrame = cap.grab();        
        if( !hasNextFrame )
            break;
		cap.retrieve(frame);

        /*if( !image )
        {
            /* allocate all the buffers *
            image = cvCreateImage( cvGetSize(frame), 8, 3 );
            image->origin = frame->origin;
            hsv = cvCreateImage( cvGetSize(frame), 8, 3 );
            hue = cvCreateImage( cvGetSize(frame), 8, 1 );
            mask = cvCreateImage( cvGetSize(frame), 8, 1 );
            backproject = cvCreateImage( cvGetSize(frame), 8, 1 );
            hist = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );
            histimg = cvCreateImage( cvSize(320,200), 8, 3 );
            cvZero( histimg );
        } 

        cvCopy( frame, image, 0 );
        cvCvtColor( image, hsv, CV_BGR2HSV ); */

        if( track_object )
        {
			track.setVMin(vmin);
			track.setVMax(vmax);
			track.setSMin(smin);

			if( track_object < 0 )
            {
				obt::TrainingInfo ti;
				ti.img = frame;
				obt::Rect shape = selection;
				ti.shapes.push_back(&shape);
				std::vector<obt::TrainingInfo> v;
				v.push_back(ti);
				track.train(v);                

/*                cvZero( histimg );
                bin_w = histimg->width / hdims;
                for( i = 0; i < hdims; i++ )
                {
                    int val = cvRound( cvGetReal1D(hist->bins,i)*histimg->height/255 );
                    CvScalar color = hsv2rgb(i*180.f/hdims);
                    cvRectangle( histimg, cvPoint(i*bin_w,histimg->height),
                                 cvPoint((i+1)*bin_w,histimg->height - val),
                                 color, -1, 8, 0 );
                }*/
				track_object = 1;
            }

			track.feed(frame);
			std::vector<const obt::Shape*> shapes;
			track.objectShapes(shapes);
			obt::RotatedRect* shape = (obt::RotatedRect*)shapes[0];
            track_window = (CvRect)shape->boundingRect();
            
			cv::ellipse( frame, *shape, CV_RGB(255,0,0), 3, CV_AA);
        }

        if( select_object && selection.width > 0 && selection.height > 0 )
        {
			cv::Mat roi = frame(selection);
			cv::bitwise_xor( roi, cv::Scalar(255), roi);
        }

		cv::imshow( "CamShiftDemo", frame );
      //  cvShowImage( "Histogram", histimg );

        c = cvWaitKey(10);
        if( (char) c == 27 )
            break;
        switch( (char) c )
        {
        case 'b':
            backproject_mode ^= 1;
            break;
        case 'c':
            track_object = 0;
        //    cvZero( histimg );
            break;
        /*case 'h':
            show_hist ^= 1;
            if( !show_hist )
                cvDestroyWindow( "Histogram" );
            else
                cvNamedWindow( "Histogram", 1 );
            break;*/
        default:
            ;
        }
    }

    cvDestroyWindow("CamShiftDemo");

    return 0;
}

#ifdef _EiC
main(1,"camshiftdemo.c");
#endif
