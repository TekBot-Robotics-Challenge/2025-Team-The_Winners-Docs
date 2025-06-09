// @ts-check

/**
 * Création des barres latérales personnalisées pour les sections du site.
 * Ce fichier est utilisé pour organiser les documents dans la barre latérale.
 * @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // Sidebar pour la section "About"
  aboutSidebar: [
    {
      type: 'category',
      label: 'About',
      items: [
        'about/introduction',
        'about/college',
        'about/team',
        'about/goals',
        'about/contacts',
      ],
    },
  ],

  // Sidebar pour la section "Pre-selection Tests"
  preselectionSidebar: [
    {
      type: 'category',
      label: 'Pre-selection Tests',
      items: [
        {
          type: 'category',
          label: 'Electronic',
          items: [
            'preselection/electronic/test1',
            'preselection/electronic/test2',
            'preselection/electronic/test3',
          ],
        },
        {
          type: 'category',
          label: 'Computer Science',
          items: [
            'preselection/computer/test1',
            'preselection/computer/test2',
            'preselection/computer/test3',
          ],
        },
        {
          type: 'category',
          label: 'Mechanic',
          items: [
            'preselection/mechanic/test1',
            'preselection/mechanic/test2',
            'preselection/mechanic/test3',
          ],
        },
        {
          type: 'category',
          label: 'Assembly',
          items: [
            'preselection/final/electronic',
            'preselection/final/computer',
            'preselection/final/mechanic',
            'preselection/final/assembly',
          ],
        },
      ],
    },
  ],

  // Sidebar pour la section "Robot Design"
  robotDesignSidebar: [
    {
      type: 'category',
      label: 'Robot Design',
      items: [
        'robot/electronic',
        'robot/computer',
        'robot/mechanic',
      ],
    },
  ],

  // Sidebar pour la section "Media"
  mediaSidebar: [
    {
      type: 'category',
      label: 'Media',
      items: [
        'media/photos',
        'media/videos',
      ],
    },
  ],

  // Sidebar pour la section "Links"
  linkSidebar: [
  'link',
  ],

};

export default sidebars;
