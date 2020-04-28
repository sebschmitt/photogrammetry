#include "imagepair-sequence-iterator.hpp"

namespace Scene {
    ImageSequenceIterator::ImageSequenceIterator(ImagePair *head) {
        this->nextNode = head;
    }

    bool ImageSequenceIterator::hasNext() {
        return nextNode != nullptr;
    }

    ImagePair *Scene::ImageSequenceIterator::next() {
        if (!hasNext())
            throw std::out_of_range("Iterator has no next element.");

        currentNode = nextNode;
        nextNode = nextNode->nextPair;

        return currentNode;
    }
}