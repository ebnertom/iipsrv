/*
    IIP PTL Command Handler Class Member Function

    Copyright (C) 2006-2017 Ruven Pillay.
	Author Ed Swangren, Epic Sciences

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
*/

#include "Task.h"
#include "Transforms.h"

#include <cmath>
#include <sstream>

using namespace std;


void PTL::send( Session* session, int resolution, int tile ){

  if( session->loglevel >= 3 ) (*session->logfile) << "PTL handler reached" << endl;

  Timer function_timer;


  // Time this command
  if( session->loglevel >= 2 ) command_timer.start();


  // If we have requested a rotation, remap the tile index to rotated coordinates
  if( (int)((session->view)->getRotation()) % 360 == 90 ){

  }
  else if( (int)((session->view)->getRotation()) % 360 == 270 ){

  }
  else if( (int)((session->view)->getRotation()) % 360 == 180 ){
    int num_res = (*session->image)->getNumResolutions();
    unsigned int im_width = (*session->image)->image_widths[num_res-resolution-1];
    unsigned int im_height = (*session->image)->image_heights[num_res-resolution-1];
    unsigned int tw = (*session->image)->getTileWidth();    
    int ntiles = (int) ceil( (double)im_width/tw ) * (int) ceil( (double)im_height/tw );
    tile = ntiles - tile - 1;
  }


  // Sanity check
  if( (resolution<0) || (tile<0) ){
    ostringstream error;
    error << "PTL :: Invalid resolution/tile number: " << resolution << "," << tile;
    throw error.str();
  }

  TileManager tilemanager( session->tileCache, *session->image, session->watermark, session->jpeg, session->png, session->logfile, session->loglevel );

  CompressionType ct;

  // Request uncompressed tile if raw pixel data is required for processing
  if( (*session->image)->getNumBitsPerPixel() > 8 || (*session->image)->getColourSpace() == CIELAB
      || (*session->image)->getNumChannels() == 2 || (*session->image)->getNumChannels() > 3
      || ( session->view->colourspace==GREYSCALE && (*session->image)->getNumChannels()==3 &&
	   (*session->image)->getNumBitsPerPixel()==8 )
      || session->view->floatProcessing()
      || session->view->getRotation() != 0.0 || session->view->flip != 0
      ){ 
    ct = UNCOMPRESSED;
  }
  else{ 
    ct = PNG;
  }

  RawTile rawtile = tilemanager.getTile( resolution, tile, session->view->xangle,
					 session->view->yangle, session->view->getLayers(), ct );


  int len = rawtile.dataLength;

  if( session->loglevel >= 2 ){
    *(session->logfile) << "PTL :: Tile size: " << rawtile.width << " x " << rawtile.height << endl
			<< "PTL :: Channels per sample: " << rawtile.channels << endl
			<< "PTL :: Bits per channel: " << rawtile.bpc << endl
			<< "PTL :: Data size is " << len << " bytes" << endl;
  }


  // Convert CIELAB to sRGB
  if( (*session->image)->getColourSpace() == CIELAB ){

    if( session->loglevel >= 4 ){
      *(session->logfile) << "PTL :: Converting from CIELAB->sRGB";
      function_timer.start();
    }
    filter_LAB2sRGB( rawtile );
    if( session->loglevel >= 4 ){
      *(session->logfile) << " in " << function_timer.getTime() << " microseconds" << endl;
    }
  }


  // Only use our float pipeline if necessary
  if( rawtile.bpc > 8 || session->view->floatProcessing() ){

    // Apply normalization and float conversion
    if( session->loglevel >= 4 ){
      *(session->logfile) << "PTL :: Normalizing and converting to float";
      function_timer.start();
    }
    filter_normalize( rawtile, (*session->image)->max, (*session->image)->min );
    if( session->loglevel >= 4 ){
      *(session->logfile) << " in " << function_timer.getTime() << " microseconds" << endl;
    }


    // Apply hill shading if requested
    if( session->view->shaded ){
      if( session->loglevel >= 4 ){
	*(session->logfile) << "PTL :: Applying hill-shading";
	function_timer.start();
      }
      filter_shade( rawtile, session->view->shade[0], session->view->shade[1] );
      if( session->loglevel >= 4 ){
	*(session->logfile) << " in " << function_timer.getTime() << " microseconds" << endl;
      }
    }


    // Apply color twist if requested
    if( session->view->ctw.size() ){
      if( session->loglevel >= 4 ){
	*(session->logfile) << "PTL :: Applying color twist";
	function_timer.start();
      }
      filter_twist( rawtile, session->view->ctw );
      if( session->loglevel >= 4 ){
	*(session->logfile) << " in " << function_timer.getTime() << " microseconds" << endl;
      }
    }


    // Apply any gamma correction
    if( session->view->getGamma() != 1.0 ){
      float gamma = session->view->getGamma();
      if( session->loglevel >= 4 ){
	*(session->logfile) << "PTL :: Applying gamma of " << gamma;
	function_timer.start();
      }
      filter_gamma( rawtile, gamma);
      if( session->loglevel >= 4 ){
	*(session->logfile) << " in " << function_timer.getTime() << " microseconds" << endl;
      }
    }


    // Apply inversion if requested
    if( session->view->inverted ){
      if( session->loglevel >= 4 ){
	*(session->logfile) << "PTL :: Applying inversion";
	function_timer.start();
      }
      filter_inv( rawtile );
      if( session->loglevel >= 4 ){
	*(session->logfile) << " in " << function_timer.getTime() << " microseconds" << endl;
      }
    }


    // Apply color mapping if requested
    if( session->view->cmapped ){
      if( session->loglevel >= 4 ){
	*(session->logfile) << "PTL :: Applying color map";
	function_timer.start();
      }
      filter_cmap( rawtile, session->view->cmap );
      if( session->loglevel >= 4 ){
	*(session->logfile) << " in " << function_timer.getTime() << " microseconds" << endl;
      }
    }


    // Apply any contrast adjustments and/or clip to 8bit from 16 or 32 bit
    float contrast = session->view->getContrast();
    if( session->loglevel >= 4 ){
      *(session->logfile) << "PTL :: Applying contrast of " << contrast << " and converting to " << session->view->getBitDepth() << " bit";
      function_timer.start();
    }
    filter_contrast( rawtile, contrast, session->view->getBitDepth() );
    if( session->loglevel >= 4 ){
      *(session->logfile) << " in " << function_timer.getTime() << " microseconds" << endl;
    }
  }

  // Reduce to 1 or 3 bands if we have an alpha channel or a multi-band image
  if( rawtile.channels == 2 || rawtile.channels > 3 ){
    unsigned int bands = (rawtile.channels==2) ? 1 : 3;
    if( session->loglevel >= 4 ){
      *(session->logfile) << "PTL :: Flattening channels to " << bands;
      function_timer.start();
    }
    filter_flatten( rawtile, bands );
    if( session->loglevel >= 4 ){
      *(session->logfile) << " in " << function_timer.getTime() << " microseconds" << endl;
    }
  }

  // Convert to greyscale if requested
  if( (*session->image)->getColourSpace() == sRGB && session->view->colourspace == GREYSCALE ){
    if( session->loglevel >= 4 ){
      *(session->logfile) << "PTL :: Converting to greyscale";
      function_timer.start();
    }
    filter_greyscale( rawtile );
    if( session->loglevel >= 4 ){
      *(session->logfile) << " in " << function_timer.getTime() << " microseconds" << endl;
    }
  }

  // Apply flip
  if( session->view->flip != 0 ){
    Timer flip_timer;
    if( session->loglevel >= 5 ){
      flip_timer.start();
    }

    filter_flip( rawtile, session->view->flip );

    if( session->loglevel >= 5 ){
      *(session->logfile) << "PTL :: Flipping image ";
      if( session->view->flip == 1 ) *(session->logfile) << "horizontally";
      else *(session->logfile) << "vertically";
      *(session->logfile) << " in " << flip_timer.getTime() << " microseconds" << endl;
    }
  }

  // Apply rotation - can apply this safely after gamma and contrast adjustment
  if( session->view->getRotation() != 0.0 ){
    float rotation = session->view->getRotation();
    if( session->loglevel >= 4 ){
      *(session->logfile) << "PTL :: Rotating image by " << rotation << " degrees";
      function_timer.start();
    }
    filter_rotate( rawtile, rotation );
    if( session->loglevel >= 4 ){
      *(session->logfile) << " in " << function_timer.getTime() << " microseconds" << endl;
    }
  }

  // Compress to PNG
  if( rawtile.compressionType == UNCOMPRESSED ){
    if( session->loglevel >= 4 ){
      *(session->logfile) << "PTL :: Compressing UNCOMPRESSED to JPEG";
      function_timer.start();
    }
    len = session->png->Compress( rawtile );
    if( session->loglevel >= 4 ){
      *(session->logfile) << " in " << function_timer.getTime() << " microseconds to "
                          << rawtile.dataLength << " bytes" << endl;
    }
  }

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

  if( session->out->putStr( static_cast<const char*>(rawtile.data), len ) != len ){
    if( session->loglevel >= 1 ){
      *(session->logfile) << "PTL :: Error writing jpeg tile" << endl;
    }
  }

  if( session->out->flush() == -1 ) {
    if( session->loglevel >= 1 ){
      *(session->logfile) << "PTL :: Error flushing jpeg tile" << endl;
    }
  }

  // Inform our response object that we have sent something to the client
  session->response->setImageSent();

  // Total PTL response time
  if( session->loglevel >= 2 ){
    *(session->logfile) << "PTL :: Total command time " << command_timer.getTime() << " microseconds" << endl;
  }
}
