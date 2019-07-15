#ifndef SDL_WINDOW_H
#define SDL_WINDOW_H

#include <SDL2/SDL.h>

class SDLWindow {

    SDL_Window* window;
    SDL_Renderer* renderer;

    // SDL_Texture will represent the whole screen.
    SDL_Texture* texture;

    Uint32* pixels;

    int width;
    int height;

    public:
        SDLWindow(int, int);
        void draw_pixel(int, int, int, int, int);
        void update();
        void clear();
        void show();
        int getWidth();
        int getHeight();
        ~SDLWindow();
};


#endif
