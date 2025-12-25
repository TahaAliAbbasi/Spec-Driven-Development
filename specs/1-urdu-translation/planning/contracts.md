# API Contracts: Urdu Translation Functionality

## Docusaurus Configuration Contract

### i18n Configuration
**Purpose**: Configure Docusaurus to support multiple languages including Urdu

**Location**: `Physical-AI-and-Humanoid-Robotics/docusaurus.config.ts`

**Contract**:
```typescript
i18n: {
  defaultLocale: 'en',
  locales: ['en', 'ur'],
  localeConfigs: {
    en: {
      label: 'English',
      direction: 'ltr',
    },
    ur: {
      label: 'اردو',
      direction: 'rtl',
      htmlLang: 'ur',
    },
  },
}
```

**Validation**:
- All specified locales must have corresponding content directories
- Default locale must be one of the supported locales
- Direction must be either 'ltr' or 'rtl'

## Language Switcher Component Contract

### Navbar Item Configuration
**Purpose**: Add a language switcher dropdown to the navigation bar

**Location**: `Physical-AI-and-Humanoid-Robotics/docusaurus.config.ts`

**Contract**:
```typescript
{
  type: 'localeDropdown',
  position: 'right',
}
```

**Validation**:
- Must be added to the navbar items array
- Position must be either 'left' or 'right'
- Should not conflict with existing navbar items

## Content Translation Contract

### Directory Structure
**Purpose**: Define the directory structure for translated content

**Location**: `Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/`

**Contract**:
- Must mirror the English content structure exactly
- All English markdown files must have corresponding Urdu translations
- File names must match exactly between languages
- Frontmatter must be properly configured for each translation

**Validation**:
- Every English content file must have a corresponding Urdu file
- File paths must be identical between languages
- All links within translated content must point to translated pages

## RTL CSS Contract

### Custom Styles
**Purpose**: Implement right-to-left text rendering for Urdu content

**Location**: `Physical-AI-and-Humanoid-Robotics/src/css/custom.css`

**Contract**:
```css
/* RTL support for Urdu */
html[dir="rtl"] body {
  direction: rtl;
  text-align: right;
}

html[dir="rtl"] .navbar__item,
html[dir="rtl"] .menu__list-item,
html[dir="rtl"] .pagination-nav__link {
  text-align: right;
}

/* Additional RTL-specific styles as needed */
```

**Validation**:
- CSS rules must be properly scoped to [dir="rtl"] attribute
- Text alignment must be right for RTL languages
- Layout elements must be mirrored appropriately

## Browser Storage Contract

### Language Preference Storage
**Purpose**: Store user's language preference in browser's localStorage

**Mechanism**: Docusaurus automatically handles language preference storage

**Contract**:
- Language preference stored in localStorage with key: `docusaurus.locale`
- Value is the locale code ('en' or 'ur')
- Persists across browser sessions
- Expires after 30 days if expiry mechanism is implemented

**Validation**:
- Storage key must match Docusaurus convention
- Value must be a valid locale code
- Storage must be accessible across all pages

## Content Synchronization Contract

### Translation Synchronization
**Purpose**: Ensure translated content stays in sync with original content

**Process**:
1. When English content is updated, corresponding Urdu content should be reviewed
2. New English content must have corresponding Urdu translations
3. Content structure changes in English must be reflected in Urdu

**Validation**:
- All content files must exist in both languages
- Content updates must trigger translation review
- Missing translations must be identified and addressed