/**
* @file Singleton.h in LooselyCouple2020_cpp
* @author rebeater
* @comment
* Create on 5/9/21 4:52 PM
* @version 1.0
**/

//
// Created by rebeater on 5/9/21.
//
#ifndef LOOSELYCOUPLE2020_CPP_SINGLETON_H
#define LOOSELYCOUPLE2020_CPP_SINGLETON_H

template<typename T>
class Singleton {
public:
    static T &Instance() {
        static T s_Instance;
        return s_Instance;
    }

protected:
    Singleton(void) {}

    ~Singleton(void) {}

private:
    Singleton(const Singleton &rhs) {}

    Singleton &operator=(const Singleton &rhs) {}
};

#endif //LOOSELYCOUPLE2020_CPP_SINGLETON_H
