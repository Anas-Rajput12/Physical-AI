// Language switcher functionality
document.addEventListener('DOMContentLoaded', function() {
  // This file is loaded async to handle language switching
  // The main logic is in the headTags script in docusaurus.config.js
  console.log('Language switcher initialized');

  // Listen for language change events to update content
  window.addEventListener('languageChanged', function(e) {
    const newLanguage = e.detail.language;
    console.log('Language changed to:', newLanguage);

    // Here you could implement logic to reload content or update translations
    // For now, just log the change
  });
});