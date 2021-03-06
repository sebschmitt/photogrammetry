#include "scene-sequence.hpp"
#include "imagepair-sequence-iterator.hpp"

#include <exception>


namespace Scene {
    void SceneSequence::append(ImagePair* node) {
        if (head == nullptr) {
            head = node;
            tail = node;
        }
        else {
            if (node->getLeftImageName() != tail->getRightImageName())
                throw std::runtime_error("The appended image pair's left image is not equal to the tail's right image.");

            tail->nextPair = node;
            node->prevPair = tail;
            tail = node;
        }
    }


    void SceneSequence::prepend(ImagePair* node) {
        if (head == nullptr) {
            head = node;
            tail = node;
        }
        else {
            if (node->getRightImageName() != head->getLeftImageName())
                throw std::runtime_error("The appended image pair's right image is not equal to the head's left image.");

            node->nextPair = head;
            head->prevPair = node;
            head = node;
        }
    }

    ImagePair SceneSequence::at(size_t index) {

        ImagePair* currentNode = head;
        for (size_t i = 0; i < index; i++) {
            if (currentNode->nextPair == nullptr)
                throw std::out_of_range("SceneSequence out of range.");
        }
        return *currentNode;
    }

    Iterator<ImagePair>* SceneSequence::createIterator() {
        return new ImageSequenceIterator(this->head);
    }
}