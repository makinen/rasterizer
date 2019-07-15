#ifndef TEXTURE_H
#define TEXTURE_H

#include <string>
#include <array>
#include <SDL2/SDL.h>

class Texture {

  std::string filename;
  SDL_Surface* image;

  public:
    Texture(const std::string& filename);
    int get_width() const;
    int get_height() const;
    std::array<int, 3> get_texel(int y, int x) const;
};

#endif
