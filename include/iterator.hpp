//
// Created by Sebastian Schmitt on 08.04.2020.
//

#ifndef YAPGT_ITERATOR_H
#define YAPGT_ITERATOR_H

namespace Scene {
    template<typename T> class Iterator {
    public:
        virtual ~Iterator() {}
        virtual T currentItem() = 0;
        virtual T next() = 0;
        virtual T first() = 0;
        virtual bool isDone() = 0;
    };
}

#endif //YAPGT_ITERATOR_H
