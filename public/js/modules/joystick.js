 // Import nipplejs using the global window object
import '/js/lib/nipplejs.min.js';

// Export the library as a named export
export const nipplejs = window.nipplejs;

// Clean up the global namespace by removing the library from the window object
delete window.nipplejs;