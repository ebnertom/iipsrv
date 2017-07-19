/*
    Image Transforms

    Copyright (C) 2004-2013 Ruven Pillay.

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


#ifndef _TRANSFORMS_H
#define _TRANSFORMS_H

#include <vector>
#include <limits>
//#include "Time.h"
#include "RawTile.h"

/// Function to create normalized array
/** @param in tile data to be adjusted
    @param min : vector of minima
    @param max : vector of maxima
*/
void filter_normalize( RawTile& in, std::vector<float>& max, std::vector<float>& min );

/// Function to apply colormap to gray images
///   based on the routine colormap.cpp in Imagin Raytracer by Olivier Ferrand
///   http://www.imagin-raytracer.org
/** @param in tile data to be converted
    @param cmap color map to apply.
*/
enum cmap_type { HOT, COLD, JET, BLUE, GREEN, RED };
void filter_cmap( RawTile& in, enum cmap_type cmap );

/// Function to invert colormaps
/** @param in tile data to be adjusted
*/
void filter_inv( RawTile& in );


/// Hillshading function to simulate raking light images
/** @param in tile input data containing normal vectors at each point
    @param h_angle angle in the horizontal plane from  12 o'clock in degrees
    @param v_angle angle in the vertical plane in degrees. 0 is flat, 90 pointing directly down.
*/
void filter_shade( RawTile& in, int h_angle, int v_angle );


/// Convert from CIELAB to sRGB colour space
/** @param in tile data to be converted */
void filter_LAB2sRGB( RawTile& in );


/// Function to apply a contrast adjustment and clip to 8 bit
/** @param in tile data to be adjusted
    @param c contrast value
*/
void filter_contrast( RawTile& in, float c, int outBpc = 8 );

template<class P>
void filter_contrast( RawTile& in, float c){
	unsigned long np = in.width * in.height * in.channels;
	P* buffer = new P[np];
	float* infptr = static_cast<float*>(in.data);
	float max_p = std::numeric_limits<P>::max();

#if defined(__ICC) || defined(__INTEL_COMPILER)
#pragma ivdep
#elif defined(_OPENMP)
#pragma omp parallel for
#endif
	for( unsigned long n=0; n<np; n++ ){
		float v = infptr[n] * max_p * c;
		v = (v > 0.0) ? floor(v + 0.5) : ceil(v - 0.5);
		buffer[n] = static_cast<P>( (v<max_p) ? (v<0.0? 0.0 : v) : max_p );
	}

	// Replace original buffer with new
	delete[] (float*) in.data;
	in.data = buffer;	
	in.bpc = sizeof(P) * 8;
	in.dataLength = np * sizeof P;
}

/// Apply a gamma correction
/** @param in tile input data
    @param g gamma
*/
void filter_gamma( RawTile& in, float g );


/// Resize image using nearest neighbour interpolation
/** @param in tile input data
    @param w target width
    @param h target height
	@param out_bpc output bit depth
*/
void filter_interpolate_nearestneighbour( RawTile& in, unsigned int w, unsigned int h, int out_bpc );

/// Resize image using nearest neighbour interpolation
/** @param in tile input data
    @param w target width
    @param h target height	
*/
template<class P>
void filter_interpolate_nearestneighbour( RawTile& in, unsigned int resampled_width, unsigned int resampled_height ){ // ed todo: test

  // Pointer to input buffer
  P* input = (P*) in.data;

  int channels = in.channels;
  unsigned int width = in.width;
  unsigned int height = in.height;

  // Pointer to output buffer
  P* output;

  // Create new buffer if size is larger than input size
  bool new_buffer = false;
  if( resampled_width*resampled_height > in.width*in.height ){
    new_buffer = true;
    output = new P[resampled_width*resampled_height*in.channels];
  }
  else output = (P*) in.data;

  // Calculate our scale
  float xscale = (float)width / (float)resampled_width;
  float yscale = (float)height / (float)resampled_height;

  for( unsigned int j=0; j<resampled_height; j++ ){
    for( unsigned int i=0; i<resampled_width; i++ ){

      // Indexes in the current pyramid resolution and resampled spaces
      // Make sure to limit our input index to the image surface
      unsigned int ii = (unsigned int) floorf(i*xscale);
      unsigned int jj = (unsigned int) floorf(j*yscale);
      unsigned int pyramid_index = (unsigned int) channels * ( ii + jj*width );

      unsigned int resampled_index = (i + j*resampled_width)*channels;
      for( int k=0; k<in.channels; k++ ){
	    output[resampled_index+k] = input[pyramid_index+k];
      }
    }
  }

  // Delete original buffer
  if( new_buffer ) delete[] (P*) input;

  // Correctly set our Rawtile info
  in.width = resampled_width;
  in.height = resampled_height;
  in.dataLength = resampled_width * resampled_height * channels * sizeof(P)/8;
  in.data = output;
}

/// Resize image using bilinear interpolation
/** @param in tile input data
    @param w target width
    @param h target height
	@param out_bpc output bit depth
*/
void filter_interpolate_bilinear( RawTile& in, unsigned int w, unsigned int h, int out_bpc );  

/// Resize image using bilinear interpolation
/** @param in tile input data
    @param w target width
    @param h target height	
*/
template<class P>
void filter_interpolate_bilinear( RawTile& in, unsigned int resampled_width, unsigned int resampled_height ){ // ed todo: test

  // Pointer to input buffer
  P* input = (P*) in.data;

  int channels = in.channels;
  unsigned int width = in.width;
  unsigned int height = in.height;

  // Create new buffer and pointer for our output
  P* output = new P[resampled_width*resampled_height*in.channels];

  // Calculate our scale
  float xscale = (float)(width) / (float)resampled_width;
  float yscale = (float)(height) / (float)resampled_height;


  // Do not parallelize for small images (256x256 pixels) as this can be slower that single threaded
#if defined(__ICC) || defined(__INTEL_COMPILER)
#pragma ivdep
#elif defined(_OPENMP)
#pragma omp parallel for if( resampled_width*resampled_height > PARALLEL_THRESHOLD )
#endif
  for( unsigned int j=0; j<resampled_height; j++ ){

    // Index to the current pyramid resolution's top left pixel
    int jj = (int) floor( j*yscale );

    // Calculate some weights - do this in the highest loop possible
    float jscale = j*yscale;
    float c = (float)(jj+1) - jscale;
    float d = jscale - (float)jj;

    for( unsigned int i=0; i<resampled_width; i++ ){

      // Index to the current pyramid resolution's top left pixel
      int ii = (int) floor( i*xscale );

      // Calculate the indices of the 4 surrounding pixels
      unsigned int p11, p12, p21, p22;
      p11 = (unsigned int) ( channels * ( ii + jj*width ) );
      p12 = (unsigned int) ( channels * ( ii + (jj+1)*width ) );
      p21 = (unsigned int) ( channels * ( (ii+1) + jj*width ) );
      p22 = (unsigned int) ( channels * ( (ii+1) + (jj+1)*width ) );

      // Calculate the rest of our weights
      float iscale = i*xscale;
      float a = (float)(ii+1) - iscale;
      float b = iscale - (float)ii;

      // Output buffer index
      unsigned int resampled_index = j*resampled_width*in.channels + i*in.channels;

      for( int k=0; k<in.channels; k++ ){
        float tx = input[p11+k]*a + input[p21+k]*b;
        float ty = input[p12+k]*a + input[p22+k]*b;
        P r = (P)( c*tx + d*ty );
        output[resampled_index+k] = r;
      }
    }
  }

  // Delete original buffer
  delete[] (P*) input;

  // Correctly set our Rawtile info
  in.width = resampled_width;
  in.height = resampled_height;
  in.dataLength = resampled_width * resampled_height * channels * sizeof(P)/8;
  in.data = output;
}


/// Rotate image - currently only by 90, 180 or 270 degrees, other values will do nothing
/** @param in tile input data
    @param angle angle of rotation - currently only rotations by 90, 180 and 270 degrees
    are suported, for other values, no rotation will occur
*/
void filter_rotate( RawTile& in, float angle );


/// Convert image to grayscale
/** @param in input image */
void filter_greyscale( RawTile& in );


/// Apply a color twist
/** @param in input image
    @param ctw 2D color twist matrix
*/
void filter_twist( RawTile& in, const std::vector< std::vector<float> >& ctw );


/// Extract bands
/** @param in input image
    @param bands number of bands
*/
void filter_flatten( RawTile& in, int bands, int bpc );

// Flatten a multi-channel image to a given number of bands by simply stripping
// away extra bands
template<class P>
void filter_flatten( RawTile& in, int bands ){

  // We cannot increase the number of channels
  if( bands >= in.channels ) return;

  unsigned long np = in.width * in.height;
  unsigned long ni = 0;
  unsigned long no = 0;
  unsigned int gap = in.channels - bands;

  // Simply loop through assigning to the same buffer
  for( unsigned long i=0; i<np; i++ ){
    for( int k=0; k<bands; k++ ){
      ((P*)in.data)[ni++] = ((P*)in.data)[no++];
    }
    no += gap;
  }

  in.channels = bands;
  in.dataLength = ni * sizeof(P)/8;
}


///Flip image
/** @param in input image
    @param o orientation (0=horizontal,1=vertical)
*/
void filter_flip( RawTile& in, int o );


#endif
