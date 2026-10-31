// The docs site's configuration: everything docs-kit does not default.
// https://github.com/branes-ai/docs-kit
export default {
  repo: 'anatomy',
  title: 'Anatomy',
  description: 'The Branes.AI mission framework: the body plan every mission composes its vehicle with, from data contracts and slots to the replay log and the closed-loop simulator.',
  accent: {
    dark: { low: '#2a1d4a', accent: '#9b7bff', high: '#ddd2ff' },
    light: { low: '#e6deff', accent: '#5b3fc4', high: '#241452' },
  },
  api: {
    name: 'Anatomy C++ API',
    brief: 'branes::anatomy — the Branes.AI mission framework',
    input: ['sdk/include', 'yaml/include', 'mobility/include', 'glue/include', 'sim/include'],
  },
};
