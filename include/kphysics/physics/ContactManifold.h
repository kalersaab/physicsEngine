#pragma once

#include <array>
#include <cstddef>

#include "kphysics/physics/Contact.h"

namespace kp {

class ContactManifold {
public:
    static constexpr std::size_t MaxContacts = 4;

    ContactManifold();

    void clear();

    bool addContact(const Contact& contact);

    std::size_t getContactCount() const;

    const Contact& getContact(std::size_t index) const;
    Contact& getContact(std::size_t index);

    const std::array<Contact, MaxContacts>&
    getContacts() const;

private:
    std::array<Contact, MaxContacts> contacts_;
    std::size_t contactCount_;
};

}