/**
 * Host-build mock of ArxTypeTraits.h.
 *
 * ArxTypeTraits backports <type_traits> and <utility> to AVR toolchains
 * that lack them. On a host compiler we just include the real STL headers.
 */

#ifndef ARX_TYPETRAITS_H
#define ARX_TYPETRAITS_H

#include <type_traits>
#include <utility>

#endif
