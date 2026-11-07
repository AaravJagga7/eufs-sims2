#ifndef INCLUDE_EUFS_SIM2_CONCEPT_MESSAGE_CONVERTER_HPP_
#define INCLUDE_EUFS_SIM2_CONCEPT_MESSAGE_CONVERTER_HPP_

namespace eufs::sim2::type {

template <typename Converter>
concept MessageConverter = requires(Converter) {
  { Converter::Convert };  // NOLINT
};  // NOLINT

}  // namespace eufs::sim2::type

#endif  // INCLUDE_EUFS_SIM2_CONCEPT_MESSAGE_CONVERTER_HPP_
