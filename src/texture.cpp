#include <string>
#include <stdexcept>
#include <iostream>
#include <array>
#include "SDL2/SDL.h"
#include "texture.h"
#include "SDL2/SDL_image.h"

Texture::Texture(const std::string& filename)
   :    filename(filename) {

  //this->image = SDL_LoadBMP(this->filename.c_str());
  this->image = IMG_Load(this->filename.c_str());

  if (this->image == NULL) {
    throw std::invalid_argument(SDL_GetError());
  }
};

int Texture::get_width() const {
  return this->image->w;
}

int Texture::get_height() const {
  return this->image->h;
}

std::array<int, 3> Texture::get_texel(int y, int x) const {

  SDL_PixelFormat *fmt;
  Uint32 temp, pixel;
  Uint8 red, green, blue, alpha;

  fmt=this->image->format;

  int bpp = fmt->BytesPerPixel;

  SDL_LockSurface(this->image);

  pixel = *static_cast<Uint32*>(this->image->pixels + y * this->image->pitch + x * bpp);

  SDL_UnlockSurface(this->image);

  /* Get Red component */
  temp = pixel & fmt->Rmask;      /* Isolate red component */
  temp = temp >> fmt->Rshift;     /* Shift it down to 8-bit */
  temp = temp << fmt->Rloss;      /* Expand to a full 8-bit number */
  red = static_cast<Uint8>(temp);

  /* Get Green component */
  temp = pixel & fmt->Gmask;      /* Isolate green component */
  temp = temp >> fmt->Gshift;     /* Shift it down to 8-bit */
  temp = temp << fmt->Gloss;      /* Expand to a full 8-bit number */
  green = static_cast<Uint8>(temp);

  /* Get Blue component */
  temp = pixel & fmt->Bmask;      /* Isolate blue component */
  temp = temp >> fmt->Bshift;     /* Shift it down to 8-bit */
  temp = temp << fmt->Bloss;      /* Expand to a full 8-bit number */
  blue = static_cast<Uint8>(temp);

  std::array<int, 3> rgb = {red, green, blue};
  return rgb;

}
