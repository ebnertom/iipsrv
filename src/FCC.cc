#define NOMINMAX

#include <string>
#include <memory>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include "Task.h"
#include "TileManager.h"

using namespace std;

void FCC::send( Session *session, const std::vector<fcc_color> &colors )
{ 
  if( session->loglevel >= 2 ) {
    *(session->logfile) << "FCC handler reached" << endl;
  }  

  if( session->view->getBitDepth() != 8 ) {
    throw string( "unsupported format: FCC supports 8bpp output only" );
  }

  Timer function_timer;
  this->session = session;
  checkImage();  

  // Time this command
  if( session->loglevel >= 2 ) {
    command_timer.start();
  }

  unsigned int resampled_width, resampled_height;
  std::vector<RawTile> complete_images;
  // todo: copy RawTiles for now, but define move constructor later

  for( int i = 0; i < session->images.size(); ++i ) {
    IIPImage *image = session->images[i];    

    if( image->getColourSpace() != GREYSCALE || image->channels != 1 || image->bpc != 16 ){      
      throw string( "FCC :: only 16bpp grayscale images supported" );
    }  

    // Calculate the number of tiles at the requested resolution
    unsigned int im_width = image->getImageWidth();
    unsigned int im_height = image->getImageHeight();
    int num_res = image->getNumResolutions();

    // Setup our view with some basic info
    session->view->setImageSize( im_width, im_height );
    session->view->setMaxResolutions( num_res );

    // Get the resolution, width and height for this view
    int requested_res = session->view->getResolution();
    im_width = image->image_widths[num_res-requested_res-1];
    im_height = image->image_heights[num_res-requested_res-1];

    if( session->loglevel >= 3 ) {
      *(session->logfile) << "FCC :: Using resolution " << requested_res << " with size " << im_width << "x" << im_height << endl;
    }

    // Data length
    int len;

    // Set up our final image sizes and if we have a region defined,
    // calculate our viewport    
    unsigned int view_left, view_top, view_width, view_height;

    if( session->view->viewPortSet() ){
      // Set the absolute viewport size and extract the co-ordinates
      view_left = session->view->getViewLeft();
      view_top = session->view->getViewTop();
      view_width = session->view->getViewWidth();
      view_height = session->view->getViewHeight();
      resampled_width = session->view->getRequestWidth();
      resampled_height = session->view->getRequestHeight();

      if( session->loglevel >= 3 ){
	*(session->logfile) << "FCC :: Region: " << view_left << "," << view_top
			    << "," << view_width << "," << view_height << endl;
      }
    }
    else{
      if( session->loglevel >= 4 ) {
	*(session->logfile) << "FCC :: No view port set" << endl;
      }

      view_left = 0;
      view_top = 0;
      view_width = im_width;
      view_height = im_height;
      resampled_width = session->view->getRequestWidth();
      resampled_height = session->view->getRequestHeight();
    }

    // If we have requested that upscaling of images be prevented adjust requested size accordingly
    // N.B. im_width and height here are from the requested resolution and not the max resolution
    if( !session->view->allow_upscaling ){
      if(resampled_width > im_width) resampled_width = im_width;
      if(resampled_height > im_height) resampled_height = im_height;
    }

    // If we have requested that the aspect ratio be maintained, make sure the final image fits *within* the requested size.
    // Don't adjust images if we have less than 0.5% difference as this is often due to rounding in resolution levels
    if( session->view->maintain_aspect ){
      float ratio = ((float)resampled_width/(float)view_width) / ((float)resampled_height/(float)view_height);
      if( ratio < 0.995 ){
	resampled_height = (unsigned int) round((((float)resampled_width/(float)view_width) * (float)view_height));
      }
      else if( ratio > 1.005 ){
	resampled_width = (unsigned int) round((((float)resampled_height/(float)view_height) * (float)view_width));
      }
    }


    if( session->loglevel >= 3 ){
      *(session->logfile) << "FCC :: Requested scaled region size is " << resampled_width << "x" << resampled_height
			  << " at resolution " << requested_res
			  << ". Nearest existing resolution has region with size " << view_width << "x" << view_height << endl;
    }

#ifdef HAVE_PNG    
    TileManager tilemanager( session->tileCache, image, session->watermark, session->jpeg, session->png, session->logfile, session->loglevel );
#else
    TileManager tilemanager( session->tileCache, image, session->watermark, session->jpeg, session->logfile, session->loglevel );
#endif

    RawTile complete_image = tilemanager.getRegion( requested_res,
						    session->view->xangle, session->view->yangle,
						    session->view->getLayers(),
						    view_left, view_top, view_width, view_height ); 

    if( complete_image.compressionType != UNCOMPRESSED ) {
      throw string( "FCC :: retrieved image data already compressed, uncompressed data buffer required" );
    }

    bool do_offset = Environment::getDoDcOffset();
    if( do_offset ) {
      // subtract the mean intensity to account for background noise            
      filter_dcoffset<uint16_t>( complete_image );
    }

    // Only use our floating point pipeline if necessary at this point 
    if( complete_image.bpc > 8 || session->view->floatProcessing() ){
      // Apply normalization and perform float conversion
      if( session->loglevel >= 5 ){
	function_timer.start();
      }

      // we want composites to be normalized to the min/max 
      // intensity within the image for viewing so that dim
      // channels aren't completely drowned out.
      float min = std::pow(2.0f, complete_image.bpc) - 1;
      float max = 0;
      const uint16_t *p = static_cast<uint16_t*>(complete_image.data);

      for (int i = 0; i < complete_image.width * complete_image.height; ++i) {
	if (p[i] < min) min = static_cast<float>(p[i]);
	if (p[i] > max) max = static_cast<float>(p[i]);
      }
	
      std::vector<float> max_v;
      std::vector<float> min_v;
      max_v.push_back(max);
      min_v.push_back(min);
		
      filter_normalize( complete_image, max_v, min_v );	
	
      if( session->loglevel >= 5 ){
	*(session->logfile) << "FCC :: Converted to floating point and normalized in "
		<< function_timer.getTime() << " microseconds" << endl;	
      }

      // Apply hill shading if requested
      if( session->view->shaded ){
	if( session->loglevel >= 5 ) function_timer.start();
	filter_shade( complete_image, session->view->shade[0], session->view->shade[1] );
	if( session->loglevel >= 5 ){
	  *(session->logfile) << "FCC :: Applying hill-shading in " << function_timer.getTime() << " microseconds" << endl;
	}
      }

      // Apply color twist if requested
      if( session->view->ctw.size() ){
	if( session->loglevel >= 5 ) function_timer.start();
	filter_twist( complete_image, session->view->ctw );
	if( session->loglevel >= 5 ){
	  *(session->logfile) << "FCC :: Applying color twist in " << function_timer.getTime() << " microseconds" << endl;
	}
      }

      // Apply any gamma correction
      if( session->view->getGamma() != 1.0 ){
	float gamma = session->view->getGamma();
	if( session->loglevel >= 5 ) function_timer.start();
	filter_gamma( complete_image, gamma );
	if( session->loglevel >= 5 ){
	  *(session->logfile) << "FCC :: Applying gamma of " << gamma << " in "
			       << function_timer.getTime() << " microseconds" << endl;
	}
      }

      // Apply inversion if requested
      if( session->view->inverted ){
	if( session->loglevel >= 5 ) function_timer.start();
	filter_inv( complete_image );
	if( session->loglevel >= 5 ){
	  *(session->logfile) << "FCC :: Applying inversion in " << function_timer.getTime() << " microseconds" << endl;
	}
      }

      // Apply color mapping if requested
      if( session->view->cmapped ){
	if( session->loglevel >= 5 ) function_timer.start();
	filter_cmap( complete_image, session->view->cmap );
	if( session->loglevel >= 5 ){
	  *(session->logfile) << "FCC :: Applying color map in " << function_timer.getTime() << " microseconds" << endl;
	}
      }

      // Apply any contrast adjustments and potentially scale to the requested output bit depth    
      if( session->loglevel >= 5 ) function_timer.start();
      filter_contrast( complete_image, session->view->getContrast(), session->view->getBitDepth() );
      if( session->loglevel >= 5 ){
	*(session->logfile) << "FCC :: Applying contrast of " << session->view->getContrast()
	  << " and converting to 8bit in " << function_timer.getTime() << " microseconds" << endl;
      }
    }  	

    // Resize our image as requested. Use the interpolation method requested in the server configuration.
    //  - Use bilinear interpolation by default  
    if( (view_width != resampled_width) || (view_height != resampled_height) ){
      string interpolation_type;      
      unsigned int interpolation = Environment::getInterpolation();

      switch( interpolation ){
       case 0:
	interpolation_type = "nearest neighbour";
	filter_interpolate_nearestneighbour( complete_image, resampled_width, resampled_height );
	break;
       default:
	interpolation_type = "bilinear";
	filter_interpolate_bilinear( complete_image, resampled_width, resampled_height );
	break;
      }      
    }    

    // Apply flip
    if( session->view->flip != 0 ){      
      filter_flip( complete_image, session->view->flip );
    }

    // Apply rotation - can apply this safely after gamma and contrast adjustment
    if( session->view->getRotation() != 0.0 ){      
      float rotation = session->view->getRotation();
      filter_rotate( complete_image, rotation );

      // For 90 and 270 rotation swap width and height
      resampled_width = complete_image.width;
      resampled_height = complete_image.height;      
    }

    complete_images.push_back( complete_image );
  } // end foreach( image )
  
  const int out_Bpp = 3;
  const RawTile &tmp = complete_images[0];
  RawTile composite( 0, tmp.resolution, tmp.hSequence, tmp.vSequence, tmp.width, tmp.height, 3, 8 );
  composite.dataLength = composite.width * composite.height * out_Bpp;
  uint8_t *dst = new uint8_t[composite.dataLength]();
  composite.data = dst;  // this is cleaned up by raw tile  
  const unsigned int dst_stride = composite.width * out_Bpp;
  const unsigned int src_stride = composite.width;

  for( int i = 0; i < complete_images.size(); ++i ) {
    const RawTile &image = complete_images[i];
    const uint8_t *src = static_cast<const uint8_t*>(image.data);
    fcc_color color = colors[i];

    for( int y = 0; y < composite.height; ++y ) {
      uint8_t *rowp = dst + y * dst_stride;
      for( int x = 0; x < composite.width; ++x ) {
	// get the gray value
	auto gv = src[y * src_stride + x];

	// convert to color
	auto r = static_cast<uint8_t>(color.r * (gv / (std::pow(2.0, image.bpc) - 1)));
	auto g = static_cast<uint8_t>(color.g * (gv / (std::pow(2.0, image.bpc) - 1)));
	auto b = static_cast<uint8_t>(color.b * (gv / (std::pow(2.0, image.bpc) - 1)));

	// compute alpha
	auto ar = r / 255.0;
	auto ag = g / 255.0;
	auto ab = b / 255.0;

	// alpha blend into the composite
	rowp[x * out_Bpp] = static_cast<uint8_t>(std::min(255.0, std::max(0.0, r + (1 - ar) * rowp[x * out_Bpp])));
	rowp[x * out_Bpp + 1] = static_cast<uint8_t>(std::min(255.0, std::max(0.0, g + (1 - ag) * rowp[x * out_Bpp + 1])));
	rowp[x * out_Bpp + 2] = static_cast<uint8_t>(std::min(255.0, std::max(0.0, b + (1 - ab) * rowp[x * out_Bpp + 2])));
      }
    }
  }
  
  // Compress to JPEG  
  if( session->loglevel >= 4 ){
    *(session->logfile) << "FCC :: Compressing UNCOMPRESSED to JPEG";      
  }
  int len = session->jpeg->Compress( composite );  


#ifndef DEBUG
  char str[1024];

  snprintf( str, 1024,
	    "Server: iipsrv/%s\r\n"
	    "X-Powered-By: IIPImage\r\n"
	    "Content-Type: %s\r\n"
            "Content-Length: %d\r\n"
	    "Last-Modified: %s\r\n"
	    "%s\r\n"
	    "\r\n",
	    VERSION, session->jpeg->getMimeType().c_str(),len,(*session->image)->getTimestamp().c_str(), session->response->getCacheControl().c_str() );

  session->out->printf( str );
#endif


  if( session->out->putStr( static_cast<const char*>(composite.data), len ) != len ){
    if( session->loglevel >= 1 ){
      *(session->logfile) << "JTL :: Error writing jpeg tile" << endl;
    }
  }

  if( session->out->flush() == -1 ) {
    if( session->loglevel >= 1 ){
      *(session->logfile) << "JTL :: Error flushing jpeg tile" << endl;
    }
  }

  // Inform our response object that we have sent something to the client
  session->response->setImageSent();

  // Total response time
  if( session->loglevel >= 2 ){
    *(session->logfile) << "JTL :: Total command time " << command_timer.getTime() << " microseconds" << endl;
  }
}