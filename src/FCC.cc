#include <string>
#include <memory>
#include <cstdint>
#include "Task.h"
#include "TileManager.h"

using namespace std;

void FCC::send( Session *session, const std::vector<fcc_color> &colors )
{
  if( session->loglevel >= 2 ) {
    *(session->logfile) << "FCC handler reached" << endl;
  }

  if( (*session->image)->channels != 1 ) {
    throw string( "unsupported format: FCC supports single channel input only" );
  }

  Timer function_timer;
  this->session = session;
  checkImage();

  if( session->view->getBitDepth() != 8 ) {
    throw string( "unsupported format: FCC supports 8bpp output only" );
  }

  // Time this command
  if( session->loglevel >= 2 ) {
    command_timer.start();
  }

  unsigned int resampled_width, resampled_height;
  std::vector<RawTile> complete_images;
  // todo: copy RawTiles for now, but define move constructor later

  for( int i = 0; i < session->images.size(); ++i ) {
    IIPImage *image = session->images[i];

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
	*(session->logfile) << "CVT :: Region: " << view_left << "," << view_top
			    << "," << view_width << "," << view_height << endl;
      }
    }
    else{
      if( session->loglevel >= 4 ) {
	*(session->logfile) << "CVT :: No view port set" << endl;
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

    // Convert CIELAB to sRGB
    if( image->getColourSpace() == CIELAB ){      
      filter_LAB2sRGB( complete_image );      
    }  

    // Only use our floating point pipeline if necessary at this point 
    if( complete_image.bpc > 8 || session->view->floatProcessing() ){
   
      // Apply normalization and perform float conversion
      {
	if( session->loglevel >= 5 ){
	  function_timer.start();
	}
	filter_normalize( complete_image, (*session->image)->max, (*session->image)->min );
	if( session->loglevel >= 5 ){
	  *(session->logfile) << "FCC :: Converting to floating point and normalizing in "
		  << function_timer.getTime() << " microseconds" << endl;
	}
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
 
    // Reduce to 1 or 3 bands if we have an alpha channel or a multi-band image
    if( (complete_image.channels==2) || (complete_image.channels>3 ) ){
      int output_channels = (complete_image.channels==2)? 1 : 3;
      if( session->loglevel >= 5 ) function_timer.start();
	
      filter_flatten( complete_image, output_channels );    
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
    // todo: complete_images.emplace_back( complete_image );
    
  } // end foreach( image )

  // todo: this sets up our composite tile nicely, but it also involves an unnecessary memcpy 
  const RawTile &tmp = complete_images[0];
  RawTile composite( 0, tmp.resolution, tmp.hSequence, tmp.vSequence, tmp.width, tmp.height, 3, 8 );
  composite.dataLength = composite.width * composite.height * 3;
  uint8_t *dst = new uint8_t[composite.dataLength]();
  composite.data = dst;  // this is cleaned up by raw tile  
  unsigned int stride = composite.width;

  for( int i = 0; i < complete_images.size(); ++i ) {
    RawTile &image = complete_images[i];
    uint8_t *src = static_cast<uint8_t*>(image.data);
    fcc_color color = colors[i];

    for( int y = 0; y < composite.height; ++y ) {
      uint8_t *rowp = dst + y * stride * 3;
      for( int x = 0; x < composite.width; ++x ) {
	// get the gray value
	auto gv = src[y * stride + x];

	// convert to color
	auto r = static_cast<uint8_t>(color.r * (gv / (pow(2.0, image.bpc) - 1)));
	auto g = static_cast<uint8_t>(color.g * (gv / (pow(2.0, image.bpc) - 1)));
	auto b = static_cast<uint8_t>(color.b * (gv / (pow(2.0, image.bpc) - 1)));

	// compute alpha
	auto ar = r / 255.0;
	auto ag = g / 255.0;
	auto ab = b / 255.0;

	// alpha blend into the composite
	rowp[x * 3] = static_cast<uint8_t>(min(255, max(0, r + (1 - ar) * rowp[x * 3])));
	rowp[x * 3 + 1] = static_cast<uint8_t>(min(255, max(0, g + (1 - ag) * rowp[x * 3 + 1])));
	rowp[x * 3 + 2] = static_cast<uint8_t>(min(255, max(0, b + (1 - ab) * rowp[x * 3 + 2])));
      }
    }
  }

  // todo: DC offset
  // todo: normalize

  // todo: remove
 /* {
    ofstream ofs;
    ofs.open("D:\\red.bin", ofstream::out | ofstream::binary);
    size_t len = composite.width * composite.height;
    unsigned char *buf = new unsigned char[len];
    for (int i = 0, d = 0; i < len; ++i, d += 3) {
      buf[i] = dst[d];
    }
    ofs.write((const char*)buf, len);
    ofs.close();

    ofs.open("D:\\green.bin", ofstream::out | ofstream::binary);        
    for (int i = 0, d = 1; i < len; ++i, d += 3) {
      buf[i] = dst[d];
    }
    ofs.write((const char*)buf, len);
    ofs.close();

    ofs.open("D:\\blue.bin", ofstream::out | ofstream::binary);        
    for (int i = 0, d = 2; i < len; ++i, d += 3) {
      buf[i] = dst[d];
    }
    ofs.write((const char*)buf, len);
    ofs.close();
  }*/

  // Initialise our output compression object - this should set the header of 
  // the image as well which is immediately pushed to the client
  session->outputCompressor->InitCompression( composite, resampled_height );

  // Add any XMP metadata to the image there is any in the original 
  if( (*session->image)->getMetadata("xmp").size() > 0 ){
    if( session->loglevel >= 4 ) *(session->logfile) << "FCC :: Adding XMP metadata" << endl;
    session->outputCompressor->addXMPMetadata( (*session->image)->getMetadata("xmp") );
  }

  unsigned int len = session->outputCompressor->getHeaderSize();

#ifdef CHUNKED
  snprintf( str, 1024, "%X\r\n", len );
  if( session->loglevel >= 4 ) *(session->logfile) << "FCC :: Image Header Chunk : " << str;
  session->out->printf( str );
#endif

  if( session->out->putStr( (const char*) session->outputCompressor->getHeader(), len ) != len ){
    if( session->loglevel >= 1 ){
      *(session->logfile) << "FCC :: Error writing output image header" << endl;
    }
  }

#ifdef CHUNKED
  session->out->printf( "\r\n" );
#endif

  // Flush our block of data
  if( session->out->flush() == -1 ) {
    if( session->loglevel >= 1 ){
      *(session->logfile) << "FCC :: Error flushing output image data" << endl;
    }
  }

  // release the header pointer which is a no-op with JPG but important for PNG
  session->outputCompressor->finishHeader();

  // Send out the data per strip of fixed height.
  // Allocate enough memory for this plus an extra 64k for instances where compressed
  // data is greater than uncompressed - for PNG, we need to supply the output pointer
  unsigned int strip_height = 128;
  unsigned int channels = composite.channels;
  stride = resampled_width * channels * composite.getBytesPerPixel();
  unsigned int strip_size = stride * strip_height + 65636;
  unsigned char* output = new unsigned char[strip_size];    

  // there is an opportunity to go parallel with this but it would require a different approach
  // in the Compressor classes - might be worth pursuing at some point - @beaudet
  int strips = (resampled_height/strip_height) + (resampled_height%strip_height == 0 ? 0 : 1);
  for( int n = 0; n < strips; n++ ) {
    // Get the starting index for this strip of data
    unsigned char* input = &((unsigned char*)composite.data)[n*strip_height*stride];

    // The last strip may have a different height
    if( (n==strips-1) && (resampled_height%strip_height!=0) ) {
      strip_height = resampled_height % strip_height;
    }

    if( session->loglevel >= 3 ){
      *(session->logfile) << "FCC :: About to compress strip with height " << strip_height << endl;
    }

    // Compress the strip
    len = session->outputCompressor->CompressStrip( input, output, strip_size, strip_height );

    if( session->loglevel >= 3 ){
      *(session->logfile) << "FCC :: Compressed data strip length is " << len << endl;
    }

#ifdef CHUNKED
    // Send chunk length in hex
    snprintf( str, 1024, "%X\r\n", len );
    if( session->loglevel >= 4 ) *(session->logfile) << "CVT :: Chunk : " << str;
    session->out->printf( str );
#endif

    // Send this strip out to the client
    if( len != session->out->putStr( (const char*) output, len ) ){
      if( session->loglevel >= 1 ){
	*(session->logfile) << "FCC :: Error writing output image strip data: " << len << endl;
      }
    }

#ifdef CHUNKED
    // Send closing chunk CRLF
    session->out->printf( "\r\n" );
#endif

    // Flush our block of data
    if( session->out->flush() == -1 ) {
      if( session->loglevel >= 1 ){
	*(session->logfile) << "FCC :: Error flushing output image data" << endl;
      }
    }
  }

  // Finish off the image compression
  len = session->outputCompressor->Finish( output, strip_size );

#ifdef CHUNKED
  snprintf( str, 1024, "%X\r\n", len );
  if( session->loglevel >= 4 ) *(session->logfile) << "FCC :: Final Data Chunk : " << str << endl;
  session->out->printf( str );
#endif

  if( session->out->putStr( (const char*) output, len ) != len ){
    if( session->loglevel >= 1 ){
      *(session->logfile) << "FCC :: Error writing output image EOI markers" << endl;
    }
  }   

  delete[] output;

#ifdef CHUNKED
  // Send closing chunk CRLF
  session->out->printf( "\r\n" );
  // Send closing blank chunk
  session->out->printf( "0\r\n\r\n" );
#endif

  if( session->out->flush()  == -1 ) {
    if( session->loglevel >= 1 ){
      *(session->logfile) << "FCC :: Error flushing output image tile" << endl;
    }
  }

  // Inform our response object that we have sent something to the client
  session->response->setImageSent();

  if( session->loglevel >= 2 ) {
      *(session->logfile) << "FCC :: Total command time " << command_timer.getTime() << " microseconds" << endl;
  }
}