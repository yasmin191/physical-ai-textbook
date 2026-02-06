import ComponentTypes from "@theme-original/NavbarItem/ComponentTypes";
import TranslateNavbarItem from "./TranslateNavbarItem";
import AuthNavbarItem from "./AuthNavbarItem";
import PersonalizeNavbarItem from "./PersonalizeNavbarItem";

export default {
  ...ComponentTypes,
  "custom-translate": TranslateNavbarItem,
  "custom-auth": AuthNavbarItem,
  "custom-personalize": PersonalizeNavbarItem,
};
