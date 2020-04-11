//
// Created by Sebastian Schmitt on 08.04.2020.
//

#ifndef YAPGT_ITERATOR_H
#define YAPGT_ITERATOR_H

    template<typename T> class Iterator {
    public:

        virtual ~Iterator() {}

        virtual T next() = 0;
        virtual bool hasNext() = 0;
        // virtual void first() = 0;
    };

#endif //YAPGT_ITERATOR_H
