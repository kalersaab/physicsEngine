#include "kphysics/physics/ContactManifold.h"

#include <stdexcept>

namespace kp {

ContactManifold::ContactManifold()
    : contactCount_(0)
{
}

void ContactManifold::clear()
{
    contactCount_ = 0;
}

bool ContactManifold::addContact(
    const Contact& contact)
{
    if (contactCount_ >= MaxContacts)
        return false;

    contacts_[contactCount_] = contact;
    ++contactCount_;

    return true;
}

std::size_t ContactManifold::getContactCount() const
{
    return contactCount_;
}

const Contact& ContactManifold::getContact(
    std::size_t index) const
{
    if (index >= contactCount_)
    {
        throw std::out_of_range(
            "ContactManifold: invalid contact index"
        );
    }

    return contacts_[index];
}

Contact& ContactManifold::getContact(
    std::size_t index)
{
    if (index >= contactCount_)
    {
        throw std::out_of_range(
            "ContactManifold: invalid contact index"
        );
    }

    return contacts_[index];
}

const std::array<Contact, ContactManifold::MaxContacts>&
ContactManifold::getContacts() const
{
    return contacts_;
}

}