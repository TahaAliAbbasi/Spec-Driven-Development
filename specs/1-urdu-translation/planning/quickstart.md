# Quickstart Guide: Urdu Translation Implementation

## Overview
This guide provides step-by-step instructions to implement Urdu translation functionality in the Physical-AI-and-Humanoid-Robotics Docusaurus web book.

## Prerequisites
- Node.js and npm installed
- Docusaurus CLI installed
- Access to the project repository
- Urdu translations for the content (prepared separately)

## Step 1: Configure i18n in Docusaurus

1. Open `Physical-AI-and-Humanoid-Robotics/docusaurus.config.ts`

2. Update the i18n configuration:
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
},
```

3. Add the language switcher to the navbar items:
```typescript
// In the navbar.items array
{
  type: 'localeDropdown',
  position: 'right',
},
```

## Step 2: Create Urdu Content Directory Structure

1. Create the directory structure for Urdu translations:
```bash
mkdir -p Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/
```

2. Create subdirectories to match the English content structure:
```bash
mkdir -p Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/module1
mkdir -p Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/module2
mkdir -p Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/module3
mkdir -p Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/module4
mkdir -p Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/module5
```

## Step 3: Add Urdu Translations

1. Copy the English content files to the Urdu directory structure:
```bash
cp Physical-AI-and-Humanoid-Robotics/docs/intro.md Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/intro.md
cp Physical-AI-and-Humanoid-Robotics/docs/prerequisites.md Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/prerequisites.md
cp Physical-AI-and-Humanoid-Robotics/docs/interactive-query.md Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/interactive-query.md
# Repeat for all content files
```

2. Replace the content of each Urdu file with the appropriate Urdu translation while maintaining the same frontmatter structure.

## Step 4: Implement RTL Support

1. Open `Physical-AI-and-Humanoid-Robotics/src/css/custom.css`

2. Add RTL-specific styles:
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

html[dir="rtl"] .pagination-nav__sublabel {
  text-align: right;
}

/* Adjust for text direction in various components */
html[dir="rtl"] .alert,
html[dir="rtl"] .theme-admonition {
  text-align: right;
}

/* Add more RTL-specific styles as needed based on your content */
```

## Step 5: Build and Test

1. Navigate to the Docusaurus project directory:
```bash
cd Physical-AI-and-Humanoid-Robotics
```

2. Build the site with all locales:
```bash
npm run build
```

3. Start the development server:
```bash
npm run start
```

4. Test the language switching functionality by clicking the language dropdown in the navbar.

## Step 6: Validate Implementation

1. Verify that all content is available in both languages
2. Test that the language switcher works correctly
3. Ensure RTL text rendering works properly for Urdu
4. Check that layout elements are properly aligned in RTL mode
5. Validate that language preference persists across sessions

## Common Issues and Solutions

### Issue: Content not showing in Urdu
**Solution**: Verify that Urdu translation files exist in the correct directory structure and have the same names as the English files.

### Issue: RTL styling not applied
**Solution**: Check that the CSS has proper `[dir="rtl"]` selectors and that the HTML direction attribute is being set correctly.

### Issue: Language switcher not appearing
**Solution**: Verify that the `localeDropdown` item was added to the navbar configuration and that both 'en' and 'ur' locales are configured in the i18n section.

## Next Steps

1. Complete translations for all content pages
2. Review and refine RTL styling
3. Test accessibility features with screen readers
4. Optimize performance if needed
5. Document the translation process for future content updates