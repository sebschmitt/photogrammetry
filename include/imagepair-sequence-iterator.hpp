//
// Created by Sebastian Schmitt on 08.04.2020.
//

#ifndef YAPGT_IMAGEPAIRSEQUENCEITERATOR_H
#define YAPGT_IMAGEPAIRSEQUENCEITERATOR_H

#include "iterator.hpp"
#include "image-pair.hpp"

namespace Scene {
    class ImageSequenceIterator : public Iterator<ImagePair> {
    private:
		ImagePair* nextNode = nullptr;
		ImagePair* currentNode = nullptr;

	public:
		ImageSequenceIterator(ImagePair* head);

		bool hasNext() override;
		ImagePair *next() override;
	};
}

#endif //YAPGT_IMAGEPAIRSEQUENCEITERATOR_H
