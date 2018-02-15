/*
    IIP FIF Command Handler Class Member Function

    Copyright (C) 2006-2015 Ruven Pillay.

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


#include <algorithm>
#include <string>
#include <sstream>
#include <vector>
#include "Task.h"
#include "URL.h"
#include "Environment.h"
#include "TPTImage.h"

#ifdef HAVE_KAKADU
#include "KakaduImage.h"
#endif

#ifdef HAVE_OPENJPEG
#include "OpenJPEGImage.h"
#endif

#define MAXIMAGECACHE 1000  // Max number of items in image cache

using namespace std;

void FIF::run( Session* session, const string& src ){

  if( session->loglevel >= 3 ) {
    *(session->logfile) << "FIF handler reached" << endl;
  }

  // Time this command
  if( session->loglevel >= 2 ) {
    command_timer.start();
  }

  // Decode any URL-encoded characters from our path
  URL url( src );
  string argument = url.decode();

  // Filter out any ../ to prevent users by-passing any file system prefix
  unsigned int n;
  while( (n=argument.find("../")) < argument.length() ) {
    argument.erase(n, 3);
  }

  if( session->loglevel >=1 ){
    if( url.warning().length() > 0 ) {
      *(session->logfile) << "FIF :: " << url.warning() << endl;
    }
    if( session->loglevel >= 5 ){
      *(session->logfile) << "FIF :: URL decoding/filtering: " << src << " => " << argument << endl;
    }
  }

  // Create our IIPImage object
  IIPImage test;

  // Get our image pattern variable
  string filesystem_prefix = Environment::getFileSystemPrefix();

  // Get our image pattern variable
  string filename_pattern = Environment::getFileNamePattern();

  // Timestamp of cached image
  time_t timestamp = 0;


  // Put the image setup into a try block as object creation can throw an exception
  try{
    // account for multiple images (for the FCC command)
    std::stringstream ss(argument);
    std::string item;
    std::vector<string> parts;

    while (std::getline(ss, item, ',')) {
      parts.push_back(item);
    }

    *session->logfile << parts.size() << " images for output" << endl;
    
    for( auto it=parts.begin(); it != parts.end(); ++it ) {
      const string file = *it;
      IIPImage *image;

      // Check whether cache is empty
      if( session->imageCache->empty() ){
	if( session->loglevel >= 1 ) *(session->logfile) << "FIF :: Image cache initialization" << endl;
	test = IIPImage( file );
	test.setFileNamePattern( filename_pattern );
	test.setFileSystemPrefix( filesystem_prefix );
	test.Initialise();
      }
      // If not, look up our object
      else{
	// Cache Hit
	if( session->imageCache->find( file ) != session->imageCache->end() ){
	  test = (*session->imageCache)[ file ];
	  timestamp = test.timestamp;       // Record timestamp if we have a cached image
	  if( session->loglevel >= 2 ){
	    *(session->logfile) << "FIF :: Image cache hit. Number of elements: " << session->imageCache->size() << endl;
	  }
	}
	// Cache Miss
	else{
	  if( session->loglevel >= 2 ) {
	    *(session->logfile) << "FIF :: Image cache miss" << endl;
	  }
	  test = IIPImage( file );
	  test.setFileNamePattern( filename_pattern );
	  test.setFileSystemPrefix( filesystem_prefix );
	  test.Initialise();
	  // Delete items if our list of images is too long.
	  if( session->imageCache->size() >= MAXIMAGECACHE ) {
	    session->imageCache->erase( session->imageCache->begin() );
	  }
	}
      }

      /***************************************************************
	Test for different image types - only TIFF is native for now
      ***************************************************************/

      ImageFormat format = test.getImageFormat();

      if( format == TIF ){
	if( session->loglevel >= 2 ) 
	  *(session->logfile) << "FIF :: TIFF image detected" << endl;

	image = new TPTImage( test );
	
	if (it == parts.begin() ) {
	  // assign the first here to maintain backward 
	  // compatibility with commands which aren't FCC.
	  *session->image = image;
	}

	// save the image here in case this is an FCC sequence
	session->images.push_back( image );
      }
#if defined(HAVE_KAKADU) || defined(HAVE_OPENJPEG)
      else if( format == JPEG2000 ){
	if( session->loglevel >= 2 ) {
	  *(session->logfile) << "FIF :: JPEG2000 image detected" << endl;
	}
#if defined(HAVE_KAKADU)
	image = new KakaduImage( test );	
	if (it == parts.begin() ) {
	  *session->image = image;
	}
	session->images.push_back( image );
#elif defined(HAVE_OPENJPEG)
	image = new OpenJPEGImage( test );	
	if (it == parts.begin() ) {
	  *session->image = image;
	}
	session->images.push_back( image );
#endif
      }
#endif
      else throw string( "Unsupported image type: " + file );

      // Open image and update timestamp
      image->openImage();      

      // Check timestamp consistency. If cached timestamp is older, update metadata
      if( timestamp > 0 && (timestamp < image->timestamp) ){
	if( session->loglevel >= 2 ){
	  *(session->logfile) << "FIF :: Image timestamp changed: reloading metadata" << endl;
	}
	image->loadImageInfo( image->currentX, image->currentY );
      }

      // Add this image to our cache, overwriting previous version if it exists
      (*session->imageCache)[file] = *image;

      if( session->loglevel >= 3 ){
	*(session->logfile) << "FIF :: Created image" << endl;
      }

      // Set the timestamp for the reply
      session->response->setLastModified( (*session->image)->getTimestamp() );

      if( session->loglevel >= 2 ) {
	*(session->logfile) << "FIF :: Image dimensions are " << image->getImageWidth()
			    << " x " << image->getImageHeight() << endl
			    << "FIF :: Image contains " << image->channels
			    << " channel" << ((image->channels > 1) ? "s" : "") << " with "
			    << image->bpc << " bit" << ((image->bpc > 1) ? "s" : "") << " per channel" << endl;
	tm *t = gmtime( &(image->timestamp) );
	char strt[64];
	strftime( strt, 64, "%a, %d %b %Y %H:%M:%S GMT", t );
	*(session->logfile) << "FIF :: Image timestamp: " << strt << endl;
      }
    } // end foreach(file)
  }
  catch( const file_error& error ){
    // Unavailable file error code is 1 3
    session->response->setError( "1 3", "FIF" );
    throw error;
  }

  // Check whether we have had an if_modified_since header. If so, compare to our image timestamp
  if( session->headers.find("HTTP_IF_MODIFIED_SINCE") != session->headers.end() ){

    tm mod_t;
    time_t t;

    strptime( (session->headers)["HTTP_IF_MODIFIED_SINCE"].c_str(), "%a, %d %b %Y %H:%M:%S %Z", &mod_t );

    // Use POSIX cross-platform mktime() function to generate a timestamp.
    // This needs UTC, but to avoid a slow TZ environment reset for each request, we set this once globally in Main.cc
    t = mktime(&mod_t);
    if( (session->loglevel >= 1) && (t == -1) ) *(session->logfile) << "FIF :: Error creating timestamp" << endl;

    if( (*session->image)->timestamp <= t ){
      if( session->loglevel >= 2 ){
	*(session->logfile) << "FIF :: Unmodified content" << endl;
	*(session->logfile) << "FIF :: Total command time " << command_timer.getTime() << " microseconds" << endl;
      }
      throw( 304 );
    }
    else{
      if( session->loglevel >= 2 ){
	*(session->logfile) << "FIF :: Content modified since requested time" << endl;
      }
    }
  }

  // Reset our angle values
  session->view->xangle = 0;
  session->view->yangle = 90;


  if( session->loglevel >= 2 ){
    *(session->logfile)	<< "FIF :: Total command time " << command_timer.getTime() << " microseconds" << endl;
  }
}
