#include "RadioSendStack.h"

RadioSendStack::RadioSendStack() : firstElement(nullptr), lastElement(nullptr), count(0) {}

RadioSendStack::~RadioSendStack() {
    clear();
}

uint8_t RadioSendStack::getCount() const { return count; }

radioStackElement* RadioSendStack::get(uint8_t index) {
    return get(firstElement, 0, index);
}

radioStackElement* RadioSendStack::get(radioStackElement* currentElement, uint8_t currentIndex, uint8_t targetIndex) {
    if (currentIndex == targetIndex) {
        return currentElement;
    }

    return get(currentElement->next, currentIndex + 1, targetIndex);
}

radioStackElement* RadioSendStack::create(const RadioMessage& data) {
    radioStackElement* element = (radioStackElement*)malloc(sizeof(radioStackElement));
    if (element == NULL) {
        return nullptr;
    }
    count++;

    element->value = data;
    return element;
}

bool RadioSendStack::removeAt(uint8_t index) {
    count--;
    if (index == 0) {
        radioStackElement* newFirst = firstElement->next;
        free(firstElement);
        firstElement = newFirst;
        return true;
    }
    else if (index == count - 1) {
        radioStackElement* newLast = get(count - 2);
        free(lastElement);
        lastElement = newLast;
        return true;
    }
    else if (index >= count) {
        return false;
    }

    radioStackElement* previousElement = get(index - 1);
    previousElement->next = previousElement->next->next;
    free(previousElement->next);
    return true;
}

bool RadioSendStack::push(const RadioMessage& data) {
    radioStackElement* element = create(data);

    if (element == nullptr) {
        return false;
    }

    if (firstElement == nullptr) {
        firstElement = element;
        lastElement = element;
        return true;
    }
    element->next = firstElement;
    firstElement = element;
    return true;
}

bool RadioSendStack::queue(const RadioMessage& data) {
    radioStackElement* element = create(data);

    if (element == nullptr) {
        return false;
    }

    if (lastElement != nullptr) {
        lastElement->next = element;
    } else {
        firstElement = element;
    }
    lastElement = element;
    return true;
}

RadioMessage RadioSendStack::peek() {
    return peek(0);
}

RadioMessage RadioSendStack::peek(uint8_t index) {
    radioStackElement* element = get(index);
    return element->value;
}

RadioMessage RadioSendStack::pop() {
    return pop(0);
}

RadioMessage RadioSendStack::pop(uint8_t index) {
    RadioMessage value = peek(index);
    removeAt(index);
    return value;
}

void RadioSendStack::clear() {
    radioStackElement* next = firstElement;

    while (count > 0) {
        radioStackElement* current = next; 
        next = current->next;
        free(current);
        count--;
    }
}