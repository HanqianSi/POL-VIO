//
// Created by leo on 19-7-14.
//

#include "line_feature.h"

//线特征提取
void LineFeature::detectLineFeatures( cv::Mat img, vector<cv::Vec4f> &lines )
{

    // Detect line features
    lines.clear();
    
    if( Config::hasLines() )//是否使用线分割
    {

        if( Config::useFLDLines() )//是否使用FLD直线
        {
        //      Ptr<line_descriptor::LSDDetectorC> lsd_ = line_descriptor::LSDDetectorC::createLSDDetectorC();
        //     // lsd parameters
        //     line_descriptor::LSDDetectorC::LSDOptions opts;
        //     opts.refine       = 1;     //1     	The way found lines will be refined
        //     opts.scale        = 0.5;   //0.8   	The scale of the image that will be used to find the lines. Range (0..1].
        //     opts.sigma_scale  = 0.6;	//0.6  	Sigma for Gaussian filter. It is computed as sigma = _sigma_scale/_scale.
        //     opts.quant        = 2.0;	//2.0   Bound to the quantization error on the gradient norm
        //     opts.ang_th       = 22.5;	//22.5	Gradient angle tolerance in degrees
        //     opts.log_eps      = 1.0;	//0		Detection threshold: -log10(NFA) > log_eps. Used only when advance refinement is chosen
        //     opts.density_th   = 0.6;	//0.7	Minimal density of aligned region points in the enclosing rectangle.
        //     opts.n_bins       = 1024;	//1024 	Number of bins in pseudo-ordering of gradient modulus.
        //     double min_line_length = 0.125;  // Line segments shorter than that are rejected
        //     // opts.refine       = 1;
        //     // opts.scale        = 0.5;
        //     // opts.sigma_scale  = 0.6;
        //     // opts.quant        = 2.0;
        //     // opts.ang_th       = 22.5;
        //     // opts.log_eps      = 1.0;
        //     // opts.density_th   = 0.6;
        //     // opts.n_bins       = 1024;
        //     // double min_line_length = 0.125;
        //     opts.min_length   = min_line_length;
        //     vector<cv::line_descriptor::KeyLine>  lsd, keylsd;
        // //     //void LSDDetectorC::detect( const std::vector<Mat>& images, std::vector<std::vector<KeyLine> >& keylines, int scale, int numOctaves, const std::vector<Mat>& masks ) const
        // //     lsd_->detect( img, lsd, 2, 1, opts);
        //      cv::Ptr<cv::line_descriptor::LSDDetectorC> lsd = cv::line_descriptor::LSDDetectorC::createLSDDetectorC();
        //     // lsd parameters
        //     cv::line_descriptor::LSDDetectorC::LSDOptions opts;
        //     opts.refine       = Config::lsdRefine();
        //     opts.scale        = Config::lsdScale();
        //     opts.sigma_scale  = Config::lsdSigmaScale();
        //     opts.quant        = Config::lsdQuant();
        //     opts.ang_th       = Config::lsdAngTh();
        //     opts.log_eps      = Config::lsdLogEps();
        //     opts.density_th   = Config::lsdDensityTh();
        //     opts.n_bins       = Config::lsdNBins();
        //     opts.min_length   = 0.125;
        //     lsd->detect( img, lsd, Config::lsdScale(), 1, opts);
        //     for ( int i = 0; i < (int) lsd.size(); i++ )
        //     {
        //         if( lsd[i].octave == 0 && lsd[i].lineLength >= 60)
        //         {
        //             keylsd.push_back( lsd[i] );
        //         }
        //     }
        //     for(int i=0;i<keylsd.size();i++)
        //     {
        //         cv::Vec4f lines_temp;
        //         lines_temp[0] = keylsd[i].startPointX;
        //         lines_temp[1] = keylsd[i].startPointY;
        //         lines_temp[2] = keylsd[i].endPointX;
        //         lines_temp[3] = keylsd[i].endPointY;
        //         lines.push_back(lines_temp);
        //     }
            
        }
        else//这里应该使用的就是LSD直线检测+LBD特征描述
        {
 
         }

    }
  
}

//线特征匹配
void LineFeature::matchLineFeatures(const cv::Mat &desc1, const cv::Mat &desc2, std::vector<int> &matches)
{
    StVO::match(desc1, desc2, Config::minRatio12L(), matches);
}

bool LineFeature::judgeMidPoint(cv::line_descriptor::KeyLine &cur_line, cv::line_descriptor::KeyLine &forw_line)
{
    Point2d cur_mid_point((cur_line.startPointX+cur_line.endPointX)/2, (cur_line.startPointY+cur_line.endPointY)/2);
    Point2d forw_mid_point((forw_line.startPointX+forw_line.endPointX)/2, (forw_line.startPointY+forw_line.endPointY)/2);
    double dist = sqrt(pow((cur_mid_point.x-forw_mid_point.x),2)+pow((cur_mid_point.y-forw_mid_point.y),2));
    if(dist < 30)
        return true;
    return false;
}


bool LineFeature::judgeAngle(cv::line_descriptor::KeyLine &cur_line, cv::line_descriptor::KeyLine &forw_line)
{
    double cur_theta = atan((cur_line.startPointY-cur_line.endPointY)/(cur_line.startPointX-cur_line.endPointX));
    double forw_theta = atan((forw_line.startPointY-forw_line.endPointY)/(forw_line.startPointX-forw_line.endPointX));
    double theta_diff = abs(cur_theta-forw_theta);
    if(theta_diff<15)
        return true;
    return false;
}

bool LineFeature::judgeMidPoint(cv::Vec4f &cur_line, cv::Vec4f &forw_line,float &average_distance)
{
    Point2d cur_mid_point((cur_line[0]+cur_line[2])/2, (cur_line[1]+cur_line[3])/2);
    Point2d forw_mid_point((forw_line[0]+forw_line[2])/2, (forw_line[1]+forw_line[3])/2);
    double dist = sqrt(pow((cur_mid_point.x-forw_mid_point.x),2)+pow((cur_mid_point.y-forw_mid_point.y),2));
    // double dist1 = sqrt(pow((cur_line[0]-forw_line[0]),2)+pow((cur_line[1]-forw_line[1]),2));
    //  double dist2 = sqrt(pow((cur_line[2]-forw_line[2]),2)+pow((cur_line[3]-forw_line[3]),2));
    //      double dist3 = sqrt(pow((cur_line[0]-forw_line[2]),2)+pow((cur_line[1]-forw_line[3]),2));
    //  double dist4= sqrt(pow((cur_line[2]-forw_line[0]),2)+pow((cur_line[3]-forw_line[1]),2));
    if(dist < average_distance)//||(dist1<average_distance||dist2<average_distance)||(dist3<average_distance||dist4<average_distance))
        return true;
    return false;
}


bool LineFeature::judgeAngle(cv::Vec4f &cur_line, cv::Vec4f &forw_line)
{
    double cur_theta = atan((cur_line[1]-cur_line[3])/(cur_line[0]-cur_line[2]));
    double forw_theta = atan((forw_line[1]-forw_line[3])/(forw_line[0]-forw_line[2]));
    double theta_diff = abs(cur_theta-forw_theta);
    if(theta_diff<50)
        return true;
    return false;
}
void LineFeature::lsddetect(cv::Mat img, vector<cv::Vec4f> &lines, double min_line_length)
{
    lines.clear();
    if( Config::hasLines() )//是否使用线分割
    {
        if( Config::useFLDLines() )//是否使用FLD直线
        {
                vector<cv::line_descriptor::KeyLine>  lsd_line;
                cv::Ptr<cv::line_descriptor::LSDDetectorC> lsd = cv::line_descriptor::LSDDetectorC::createLSDDetectorC();
                // lsd parameters
                cv::line_descriptor::LSDDetectorC::LSDOptions opts;
                    opts.refine       = 1;     //1     	The way found lines will be refined
                    opts.scale        = 0.5;   //0.8   	The scale of the image that will be used to find the lines. Range (0..1].
                    opts.sigma_scale  = 0.6;	//0.6  	Sigma for Gaussian filter. It is computed as sigma = _sigma_scale/_scale.
                    opts.quant        = 2.0;	//2.0   Bound to the quantization error on the gradient norm
                    opts.ang_th       = 22.5;	//22.5	Gradient angle tolerance in degrees
                    opts.log_eps      = 1.0;	//0		Detection threshold: -log10(NFA) > log_eps. Used only when advance refinement is chosen
                    opts.density_th   = 0.6;	//0.7	Minimal density of aligned region points in the enclosing rectangle.
                    opts.n_bins       = 1024;	//1024 	Number of bins in pseudo-ordering of gradient modulus.
                    double min_line_length = 0.125;  // Line segments shorter than that are rejected
                    // opts.refine       = 1;
                    // opts.scale        = 0.5;
                    // opts.sigma_scale  = 0.6;
                    // opts.quant        = 2.0;
                    // opts.ang_th       = 22.5;
                    // opts.log_eps      = 1.0;
                    // opts.density_th   = 0.6;
                    // opts.n_bins       = 1024;
                    // double min_line_length = 0.125;
                    opts.min_length   = min_line_length*(std::min(img.cols,img.rows));
                    // opts.min_length = min_line_length;
        //         opts.refine       = Config::lsdRefine();
        //         opts.scale        = Config::lsdScale();
        //         opts.sigma_scale  = Config::lsdSigmaScale();
        //         opts.quant        = Config::lsdQuant();
        //         opts.ang_th       = Config::lsdAngTh();
        //         opts.log_eps      = Config::lsdLogEps();
        //         opts.density_th   = Config::lsdDensityTh();
        //         opts.n_bins       = Config::lsdNBins();
        //         opts.min_length   = min_line_length;
                // Config::lsdScale() = 30;
                lsd->detect( img, lsd_line, Config::lsdScale(), 1, opts);
                // Config::lsdNFeatures() = 100;
                // if( lsd_line.size()>Config::lsdNFeatures() && Config::lsdNFeatures()!=0  )
                // {
                //     // sort lines by their response
                //     sort( lsd_line.begin(), lsd_line.end(), sort_lines_by_response() );
                //     //sort( lines.begin(), lines.end(), sort_lines_by_length() );
                //     lsd_line.resize(Config::lsdNFeatures());
                //     // reassign index
                //     for( int i = 0; i < Config::lsdNFeatures(); i++  )
                //         lsd_line[i].class_id = i;
                // }
                for(int i=0;i<lsd_line.size();i++)
                {
                    cv::Vec4f lines_temp;
                    lines_temp[0] = lsd_line[i].startPointX;
                    lines_temp[2] = lsd_line[i].endPointX;
                    lines_temp[1] = lsd_line[i].startPointY;
                    lines_temp[3] = lsd_line[i].endPointY;
                    lines.push_back(lines_temp);
                }
         }
        else
        {
           
                    
                    vector<cv::Vec4f> fld_lines;
                Ptr<cv::ximgproc::FastLineDetector> fld = cv::ximgproc::createFastLineDetector();
                fld->detect( img, fld_lines );

                // filter lines
                if( fld_lines.size()>Config::lsdNFeatures() && Config::lsdNFeatures()!=0  )//对线条进行滤波
                {
                    // sort lines by their response
                    sort( fld_lines.begin(), fld_lines.end(), sort_flines_by_length() );
                    fld_lines.resize(Config::lsdNFeatures());
                }

                // loop over lines object transforming into a vector<KeyLine>
                lines.reserve(fld_lines.size());
                for( int i = 0; i < fld_lines.size(); i++ )
                {
                    cv::Vec4f kl;
                     double octaveScale = 1.f;

                    kl[0]    = fld_lines[i][0] * octaveScale;
                    kl[1]     = fld_lines[i][1] * octaveScale;
                    kl[2]      = fld_lines[i][2] * octaveScale;
                    kl[3]      = fld_lines[i][3] * octaveScale;

                    lines.push_back( kl );

                }
        }
    }
}
