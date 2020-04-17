#include "imagepair-sequence-iterator.hpp"

#include <exception>

Scene::ImageSequenceIterator::ImageSequenceIterator(ImagePair* head) {
	this->nextNode = head;
}


//void Scene::ImageSequenceIterator::first() {
//	while (currentNode->prevPair != nullptr)
//	{
//		currentNode = currentNode->prevPair;
//	}
//}

bool Scene::ImageSequenceIterator::hasNext() {
	return nextNode != nullptr;
}

Scene::ImagePair *Scene::ImageSequenceIterator::next() {
	if (!hasNext())
		throw std::out_of_range("Iterator has no next element.");

	currentNode = nextNode;
	nextNode = nextNode->nextPair;

	return currentNode;
}
