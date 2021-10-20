#pragma once
#include <thread>
#include <termios.h>
#include <unistd.h>

// a class for terminal key inputs
class Keyboard {
 public:
    Keyboard() {
        struct termios new_tio;

        /* get the terminal settings for stdin */
        tcgetattr(STDIN_FILENO,&old_tio_);

        /* we want to keep the old setting to restore them a the end */
        new_tio=old_tio_;

        /* disable canonical mode (buffered i/o) and local echo */
        new_tio.c_lflag &=(~ICANON & ~ECHO);

        /* set the new settings immediately */
        tcsetattr(STDIN_FILENO,TCSANOW,&new_tio);

        strin_ = false;
        t_ = new std::thread(&Keyboard::run, this);
    }
    bool new_key() const {
        return strin_;
    }
    char get_char() {
        strin_ = false;
        return current_char_;
    }
    void run() {
        while(1) { 
            current_char_=getchar(); 
            strin_ = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    ~Keyboard() {
        pthread_cancel(t_->native_handle());
        t_->join();
        /* restore the former settings */
        tcsetattr(STDIN_FILENO,TCSANOW,&old_tio_);
    }
 private:
    struct termios old_tio_;
    char current_char_;
    bool strin_;
    std::thread *t_;
};
