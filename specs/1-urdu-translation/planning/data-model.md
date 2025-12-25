# Data Model: Urdu Translation Functionality

## Language Preference Entity

### Definition
The Language Preference entity represents the user's selected language preference stored in the browser.

### Attributes
- **languageCode** (string): The ISO 639-1/639-2 language code (e.g., 'en', 'ur')
- **timestamp** (datetime): The time when the preference was set
- **expiryDate** (datetime): The date when the preference should expire (optional)

### Validation Rules
- languageCode must be one of the supported locales ('en', 'ur')
- timestamp must be in ISO 8601 format
- expiryDate must be in the future if specified

## Translation Content Entity

### Definition
The Translation Content entity represents the translated versions of the web book content.

### Attributes
- **locale** (string): The target locale code ('ur' for Urdu)
- **originalPath** (string): The path of the original English content
- **translatedContent** (string): The translated content in the target language
- **lastUpdated** (datetime): The date when the translation was last updated

### Validation Rules
- locale must be a valid language code
- originalPath must match the existing English content structure
- translatedContent must be properly formatted markdown

## Language Switcher Component Entity

### Definition
The Language Switcher Component entity represents the UI component that allows users to switch between languages.

### Attributes
- **availableLanguages** (array): List of supported language codes
- **currentLanguage** (string): The currently selected language
- **displayLabels** (object): Language labels for UI display (e.g., {'en': 'English', 'ur': 'اردو'})

### Validation Rules
- availableLanguages must only contain supported locales
- currentLanguage must be one of the available languages
- displayLabels must have entries for all available languages

## State Transitions

### Language Preference
1. **Initial State**: No language preference set
2. **User Action**: User selects a language from the switcher
3. **Transition**: Language preference stored in localStorage
4. **Result State**: Language preference is set and remembered

### Content Display
1. **Initial State**: Content displayed in default language (English)
2. **User Action**: User selects different language
3. **Transition**: Content switches to selected language
4. **Result State**: All content displayed in selected language