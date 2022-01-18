/*
    ====================================================================
    A Smart Pointer to IMPLementation (i.e. Smart PIMPL or just SPIMPL).
    ====================================================================

    Version: 1.2

    Latest version:
        https://github.com/oliora/samples/blob/master/spimpl.h
    Rationale and description:
        http://oliora.github.io/2015/12/29/pimpl-and-rule-of-zero.html

    Copyright (c) 2015 Andrey Upadyshev (oliora@gmail.com)

    Distributed under the Boost Software License, Version 1.0.
    See http://www.boost.org/LICENSE_1_0.txt

    Changes history
    ---------------
    v1.2:
        - const correctness
    v1.1:
        - auto_ptr support is disabled by default for C++17 compatibility
    v1.0:
        - Released
 */

#ifndef ESP_CORE_SPIMPL_H_
#define ESP_CORE_SPIMPL_H_

#include <cassert>
#include <memory>
#include <type_traits>

#if defined _MSC_VER && _MSC_VER < 1900  // MS Visual Studio before VS2015
#define SPIMPL_NO_CPP11_NOEXCEPT
#define SPIMPL_NO_CPP11_CONSTEXPR
#define SPIMPL_NO_CPP11_DEFAULT_MOVE_SPEC_FUNC
#endif

#if !defined SPIMPL_NO_CPP11_NOEXCEPT
#define SPIMPL_NOEXCEPT noexcept
#else
#define SPIMPL_NOEXCEPT
#endif

#if !defined SPIMPL_NO_CPP11_CONSTEXPR
#define SPIMPL_CONSTEXPR constexpr
#else
#define SPIMPL_CONSTEXPR
#endif

// define SPIMPL_HAS_AUTO_PTR to enable constructor and assignment operator that
// accept std::auto_ptr
// TODO: auto detect std::auto_ptr support

namespace spimpl {
namespace details {
template <class T>
T* default_copy(T* src) {
  // NOLINTNEXTLINE(bugprone-sizeof-expression)
  static_assert(sizeof(T) > 0, "default_copy cannot copy incomplete type");
  static_assert(!std::is_void<T>::value,
                "default_copy cannot copy incomplete type");
  return new T(*src);
}

template <class T>
void default_delete(T* p) SPIMPL_NOEXCEPT {
  // NOLINTNEXTLINE(bugprone-sizeof-expression)
  static_assert(sizeof(T) > 0, "default_delete cannot delete incomplete type");
  static_assert(!std::is_void<T>::value,
                "default_delete cannot delete incomplete type");
  delete p;
}

template <class T>
struct default_deleter {
  using type = void (*)(T*);
};

template <class T>
using default_deleter_t = typename default_deleter<T>::type;

template <class T>
struct default_copier {
  using type = T* (*)(T*);
};

template <class T>
using default_copier_t = typename default_copier<T>::type;

template <class T, class D, class C>
struct is_default_manageable
    : public std::integral_constant<
          bool,
          std::is_same<D, default_deleter_t<T>>::value &&
              std::is_same<C, default_copier_t<T>>::value> {};
}  // namespace details

template <class T,
          class Deleter = details::default_deleter_t<T>,
          class Copier = details::default_copier_t<T>>
class impl_ptr {
 private:
  static_assert(!std::is_array<T>::value,
                "impl_ptr specialization for arrays is not implemented");
  struct dummy_t_ {
    int dummy_;
  };

 public:
  using pointer = T*;
  using const_pointer = typename std::add_const<T>::type*;
  using reference = T&;
  using const_reference = typename std::add_const<T>::type&;
  using element_type = T;
  using copier_type = typename std::decay<Copier>::type;
  using deleter_type = typename std::decay<Deleter>::type;
  using unique_ptr_type = std::unique_ptr<T, deleter_type>;
  using is_default_manageable =
      details::is_default_manageable<T, deleter_type, copier_type>;

  SPIMPL_CONSTEXPR impl_ptr() SPIMPL_NOEXCEPT : ptr_(nullptr, deleter_type{}),
                                                copier_(copier_type{}) {}

  SPIMPL_CONSTEXPR explicit impl_ptr(std::nullptr_t) SPIMPL_NOEXCEPT
      : impl_ptr() {}

  template <class D, class C>
  impl_ptr(
      pointer p,
      D&& d,
      C&& c,
      typename std::enable_if<std::is_convertible<D, deleter_type>::value &&
                                  std::is_convertible<C, copier_type>::value,
                              dummy_t_>::type = dummy_t_()) SPIMPL_NOEXCEPT
      : ptr_(std::move(p), std::forward<D>(d)),
        copier_(std::forward<C>(c)) {}

  template <class U>
  explicit impl_ptr(
      U* u,
      typename std::enable_if<std::is_convertible<U*, pointer>::value &&
                                  is_default_manageable::value,
                              dummy_t_>::type = dummy_t_()) SPIMPL_NOEXCEPT
      : impl_ptr(u, &details::default_delete<T>, &details::default_copy<T>) {}

  impl_ptr(const impl_ptr& r) : impl_ptr(r.clone()) {}

#ifndef SPIMPL_NO_CPP11_DEFAULT_MOVE_SPEC_FUNC
  impl_ptr(impl_ptr&& r) SPIMPL_NOEXCEPT = default;
#else
  impl_ptr(impl_ptr&& r) SPIMPL_NOEXCEPT : ptr_(std::move(r.ptr_)),
                                           copier_(std::move(r.copier_)) {}
#endif

#ifdef SPIMPL_HAS_AUTO_PTR
  template <class U>
  impl_ptr(std::auto_ptr<U>&& u,
           typename std::enable_if<std::is_convertible<U*, pointer>::value &&
                                       is_default_manageable::value,
                                   dummy_t_>::type = dummy_t_()) SPIMPL_NOEXCEPT
      : ptr_(u.release(), &details::default_delete<T>),
        copier_(&details::default_copy<T>) {}
#endif

  template <class U>
  explicit impl_ptr(
      std::unique_ptr<U>&& u,
      typename std::enable_if<std::is_convertible<U*, pointer>::value &&
                                  is_default_manageable::value,
                              dummy_t_>::type = dummy_t_()) SPIMPL_NOEXCEPT
      : ptr_(u.release(), &details::default_delete<T>),
        copier_(&details::default_copy<T>) {}

  template <class U, class D, class C>
  impl_ptr(
      std::unique_ptr<U, D>&& u,
      C&& c,
      typename std::enable_if<std::is_convertible<U*, pointer>::value &&
                                  std::is_convertible<D, deleter_type>::value &&
                                  std::is_convertible<C, copier_type>::value,
                              dummy_t_>::type = dummy_t_()) SPIMPL_NOEXCEPT
      : ptr_(std::move(u)),
        copier_(std::forward<C>(c)) {}

  template <class U, class D, class C>
  explicit impl_ptr(
      impl_ptr<U, D, C>&& u,
      typename std::enable_if<std::is_convertible<U*, pointer>::value &&
                                  std::is_convertible<D, deleter_type>::value &&
                                  std::is_convertible<C, copier_type>::value,
                              dummy_t_>::type = dummy_t_()) SPIMPL_NOEXCEPT
      : ptr_(std::move(u.ptr_)),
        copier_(std::move(u.copier_)) {}

  impl_ptr& operator=(const impl_ptr& r) {
    if (this == &r)
      return *this;
    // NOLINTNEXTLINE(misc-unconventional-assign-operator)
    return operator=(r.clone());
  }

#ifndef SPIMPL_NO_CPP11_DEFAULT_MOVE_SPEC_FUNC
  impl_ptr& operator=(impl_ptr&& r) SPIMPL_NOEXCEPT = default;
#else
  impl_ptr& operator=(impl_ptr&& r) SPIMPL_NOEXCEPT {
    ptr_ = std::move(r.ptr_);
    copier_ = std::move(r.copier_);
    return *this;
  }
#endif

  template <class U, class D, class C>
  // NOLINTNEXTLINE(misc-unconventional-assign-operator)
  typename std::enable_if<std::is_convertible<U*, pointer>::value &&
                              std::is_convertible<D, deleter_type>::value &&
                              std::is_convertible<C, copier_type>::value,
                          impl_ptr&>::type
  operator=(impl_ptr<U, D, C>&& u) SPIMPL_NOEXCEPT {
    ptr_ = std::move(u.ptr_);
    copier_ = std::move(u.copier_);
    return *this;
  }

  template <class U, class D, class C>
  // NOLINTNEXTLINE(misc-unconventional-assign-operator)
  typename std::enable_if<std::is_convertible<U*, pointer>::value &&
                              std::is_convertible<D, deleter_type>::value &&
                              std::is_convertible<C, copier_type>::value,
                          impl_ptr&>::type
  operator=(const impl_ptr<U, D, C>& u) {
    return operator=(u.clone());
  }

  //

#ifdef SPIMPL_HAS_AUTO_PTR
  template <class U>
  // NOLINTNEXTLINE(misc-unconventional-assign-operator)
  typename std::enable_if<std::is_convertible<U*, pointer>::value &&
                              is_default_manageable::value,
                          impl_ptr&>::type
  operator=(std::auto_ptr<U>&& u) SPIMPL_NOEXCEPT {
    return operator=(impl_ptr(std::move(u)));
  }
#endif

  template <class U>
  // NOLINTNEXTLINE(misc-unconventional-assign-operator)
  typename std::enable_if<std::is_convertible<U*, pointer>::value &&
                              is_default_manageable::value,
                          impl_ptr&>::type
  operator=(std::unique_ptr<U>&& u) SPIMPL_NOEXCEPT {
    return operator=(impl_ptr(std::move(u)));
  }

  impl_ptr clone() const {
    return impl_ptr(ptr_ ? copier_(ptr_.get()) : nullptr, ptr_.get_deleter(),
                    copier_);
  }

  reference operator*() { return *ptr_; }
  const_reference operator*() const { return *ptr_; }

  pointer operator->() SPIMPL_NOEXCEPT { return get(); }
  const_pointer operator->() const SPIMPL_NOEXCEPT { return get(); }

  pointer get() SPIMPL_NOEXCEPT { return ptr_.get(); }
  const_pointer get() const SPIMPL_NOEXCEPT { return ptr_.get(); }

  void swap(impl_ptr& u) SPIMPL_NOEXCEPT {
    using std::swap;
    ptr_.swap(u.ptr_);
    swap(copier_, u.copier_);
  }

  pointer release() SPIMPL_NOEXCEPT { return ptr_.release(); }

  unique_ptr_type release_unique() SPIMPL_NOEXCEPT { return std::move(ptr_); }

  explicit operator bool() const SPIMPL_NOEXCEPT {
    return static_cast<bool>(ptr_);
  }

  typename std::remove_reference<deleter_type>::type& get_deleter()
      SPIMPL_NOEXCEPT {
    return ptr_.get_deleter();
  }
  const typename std::remove_reference<deleter_type>::type& get_deleter() const
      SPIMPL_NOEXCEPT {
    return ptr_.get_deleter();
  }

  typename std::remove_reference<copier_type>::type& get_copier()
      SPIMPL_NOEXCEPT {
    return copier_;
  }
  const typename std::remove_reference<copier_type>::type& get_copier() const
      SPIMPL_NOEXCEPT {
    return copier_;
  }

 private:
  unique_ptr_type ptr_;
  copier_type copier_;
};

template <class T, class D, class C>
inline void swap(impl_ptr<T, D, C>& l, impl_ptr<T, D, C>& r) SPIMPL_NOEXCEPT {
  l.swap(r);
}

template <class T1, class D1, class C1, class T2, class D2, class C2>
inline bool operator==(const impl_ptr<T1, D1, C1>& l,
                       const impl_ptr<T2, D2, C2>& r) {
  return l.get() == r.get();
}

template <class T1, class D1, class C1, class T2, class D2, class C2>
inline bool operator!=(const impl_ptr<T1, D1, C1>& l,
                       const impl_ptr<T2, D2, C2>& r) {
  return !(l == r);
}

template <class T1, class D1, class C1, class T2, class D2, class C2>
inline bool operator<(const impl_ptr<T1, D1, C1>& l,
                      const impl_ptr<T2, D2, C2>& r) {
  using P1 = typename impl_ptr<T1, D1, C1>::pointer;
  using P2 = typename impl_ptr<T2, D2, C2>::pointer;
  using CT = typename std::common_type<P1, P2>::type;
  return std::less<CT>()(l.get(), r.get());
}

template <class T1, class D1, class C1, class T2, class D2, class C2>
inline bool operator>(const impl_ptr<T1, D1, C1>& l,
                      const impl_ptr<T2, D2, C2>& r) {
  return r < l;
}

template <class T1, class D1, class C1, class T2, class D2, class C2>
inline bool operator<=(const impl_ptr<T1, D1, C1>& l,
                       const impl_ptr<T2, D2, C2>& r) {
  return !(r < l);
}

template <class T1, class D1, class C1, class T2, class D2, class C2>
inline bool operator>=(const impl_ptr<T1, D1, C1>& l,
                       const impl_ptr<T2, D2, C2>& r) {
  return !(l < r);
}

template <class T, class D, class C>
inline bool operator==(const impl_ptr<T, D, C>& p,
                       std::nullptr_t) SPIMPL_NOEXCEPT {
  return !p;
}

template <class T, class D, class C>
inline bool operator==(std::nullptr_t,
                       const impl_ptr<T, D, C>& p) SPIMPL_NOEXCEPT {
  return !p;
}

template <class T, class D, class C>
inline bool operator!=(const impl_ptr<T, D, C>& p,
                       std::nullptr_t) SPIMPL_NOEXCEPT {
  return static_cast<bool>(p);
}

template <class T, class D, class C>
inline bool operator!=(std::nullptr_t,
                       const impl_ptr<T, D, C>& p) SPIMPL_NOEXCEPT {
  return static_cast<bool>(p);
}

template <class T, class D, class C>
inline bool operator<(const impl_ptr<T, D, C>& l, std::nullptr_t) {
  using P = typename impl_ptr<T, D, C>::pointer;
  return std::less<P>()(l.get(), nullptr);
}

template <class T, class D, class C>
inline bool operator<(std::nullptr_t, const impl_ptr<T, D, C>& p) {
  using P = typename impl_ptr<T, D, C>::pointer;
  return std::less<P>()(nullptr, p.get());
}

template <class T, class D, class C>
inline bool operator>(const impl_ptr<T, D, C>& p, std::nullptr_t) {
  return nullptr < p;
}

template <class T, class D, class C>
inline bool operator>(std::nullptr_t, const impl_ptr<T, D, C>& p) {
  return p < nullptr;
}

template <class T, class D, class C>
inline bool operator<=(const impl_ptr<T, D, C>& p, std::nullptr_t) {
  return !(nullptr < p);
}

template <class T, class D, class C>
inline bool operator<=(std::nullptr_t, const impl_ptr<T, D, C>& p) {
  return !(p < nullptr);
}

template <class T, class D, class C>
inline bool operator>=(const impl_ptr<T, D, C>& p, std::nullptr_t) {
  return !(p < nullptr);
}

template <class T, class D, class C>
inline bool operator>=(std::nullptr_t, const impl_ptr<T, D, C>& p) {
  return !(nullptr < p);
}

template <class T, class... Args>
inline impl_ptr<T> make_impl(Args&&... args) {
  return impl_ptr<T>(new T(std::forward<Args>(args)...),
                     &details::default_delete<T>, &details::default_copy<T>);
}

// Helpers to manage unique impl, stored in std::unique_ptr

template <class T, class Deleter = void (*)(T*)>
using unique_impl_ptr = std::unique_ptr<T, Deleter>;

template <class T, class... Args>
inline unique_impl_ptr<T> make_unique_impl(Args&&... args) {
  static_assert(!std::is_array<T>::value,
                "unique_impl_ptr does not support arrays");
  return unique_impl_ptr<T>(new T(std::forward<Args>(args)...),
                            &details::default_delete<T>);
}
}  // namespace spimpl

namespace std {
template <class T, class D, class C>
struct hash<spimpl::impl_ptr<T, D, C>> {
  using argument_type = spimpl::impl_ptr<T, D, C>;
  using result_type = size_t;

  result_type operator()(const argument_type& p) const SPIMPL_NOEXCEPT {
    return hash<typename argument_type::pointer>()(p.get());
  }
};
}  // namespace std

#endif  // ESP_CORE_SPIMPL_H_
