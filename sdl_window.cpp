#include <iostream>
#include <stdexcept>
#include <SDL2/SDL.h>
#include "sdl_window.h"

SDLWindow::SDLWindow(int width, int height)
    :   width(width), height(height) {

    SDL_Init(SDL_INIT_VIDEO);
    SDL_CreateWindowAndRenderer(width, height, 0, &window, &renderer);
    // TODO check NULLs

    // make the scaled rendering look smoother.
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");
    SDL_RenderSetLogicalSize(renderer, width, height);

    texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888,
                                SDL_TEXTUREACCESS_STREAMING, width,
                                height);
    pixels = new Uint32[width * height];
    this->clear();
}

int SDLWindow::getWidth() {
  return this->width;
}

int SDLWindow::getHeight() {
  return this->height;
}

void SDLWindow::show() {
    // clear the whole window, put the cleared window on the screen.
    SDL_RenderClear(renderer);
    SDL_RenderPresent(renderer);
}

void SDLWindow::clear() {
    // clear the pixel array
    SDL_memset(pixels, 0, width * height * sizeof(Uint32));
}

void SDLWindow::draw_pixel(int y, int x, int red, int green, int blue) {

    if (y < 0 || y >= this->height) {
        throw std::invalid_argument( "the y coordinate locates outside of the canvas" );
    }

    if (x < 0 || x >= this->width) {
        throw std::invalid_argument( "the x coordinate locates outside of the canvas" );
    }

    Uint32 format = SDL_GetWindowPixelFormat(window);
    SDL_PixelFormat* fmt = SDL_AllocFormat(format);
    Uint32 color = SDL_MapRGB(fmt, red, green, blue);

    pixels[y * this->width + x] = color;
}

void SDLWindow::update() {

    // This will upload the pixels to GPU memory.
    SDL_UpdateTexture(texture, NULL, static_cast<void*>(pixels), width * sizeof (Uint32));

    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, texture, NULL, NULL);
    SDL_RenderPresent(renderer);
}

SDLWindow::~SDLWindow() {
    delete[] pixels;
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

