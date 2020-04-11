//
// Created by Sebastian Schmitt on 08.04.2020.
//

#ifndef YAPGT_SCENE_SEQUENCE_H
#define YAPGT_SCENE_SEQUENCE_H

#include "image-pair.hpp"
#include "iterator.hpp"

namespace Scene {
    class SceneSequence {
    private:
        ImagePair *head = nullptr;
        ImagePair *tail = nullptr;
        // ImagePair *currentNode;
    public:
        void append(ImagePair *node);
        void prepend(ImagePair *node);
        ImagePair at(size_t index);
        Iterator<ImagePair>* createIterator();
    };
}

#endif //YAPGT_SCENE_SEQUENCE_H
