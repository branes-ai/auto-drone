module.exports = {
  extends: ['@commitlint/config-conventional'],
  rules: {
    // Allowed types for this project
    'type-enum': [
      2,
      'always',
      [
        'feat',     // New feature
        'fix',      // Bug fix
        'docs',     // Documentation only
        'style',    // Formatting, missing semicolons, etc. (no code change)
        'refactor', // Code change that neither fixes a bug nor adds a feature
        'perf',     // Performance improvement
        'test',     // Adding or updating tests
        'build',    // Build system or external dependencies
        'ci',       // CI/CD configuration
        'chore',    // Maintenance tasks
        'revert',   // Revert a previous commit
      ],
    ],
    // Allowed scopes for this project
    'scope-enum': [
      1, // warning, not error — new scopes are fine
      'always',
      [
        'zenoh',        // libs/zenoh_interface
        'data-types',   // libs/data_types
        'control',      // libs/control_algorithms
        'bt',           // libs/behavior_tree
        'perception',   // autonomy_stack/object_tracking
        'avoidance',    // autonomy_stack/obstacle_avoidance
        'airsim',       // sim_interfaces/airsim_*
        'isaac',        // sim_interfaces/isaac_*
        'mission',      // packages/mission_framework
        'demo',         // demos/*
        'ci',           // .github/workflows
        'docs',         // docs/*
        'cmake',        // build system
      ],
    ],
    'subject-case': [2, 'never', ['start-case', 'pascal-case', 'upper-case']],
    'header-max-length': [2, 'always', 100],
    'body-max-line-length': [1, 'always', 200], // warning only
  },
};
