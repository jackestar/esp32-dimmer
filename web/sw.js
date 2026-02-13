const CACHE_NAME = 'esp32-dimmer-v1';
const URLS_TO_CACHE = [
  '.',
  'index.html',
  'style.css',
  'manifest.json'
];

self.addEventListener('install', (event) => {
  event.waitUntil(
    caches.open(CACHE_NAME).then((cache) => cache.addAll(URLS_TO_CACHE))
  );
  self.skipWaiting();
});

self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches.keys().then((keys) => Promise.all(
      keys.filter(k => k !== CACHE_NAME).map(k => caches.delete(k))
    ))
  );
  self.clients.claim();
});

self.addEventListener('fetch', (event) => {
  event.respondWith(
    caches.match(event.request).then((response) => {
      if (response) return response;
      return fetch(event.request).then((res) => {
        // Optionally cache new resources
        return res;
      }).catch(() => {
        // Fallback: return cached index for navigation requests
        if (event.request.mode === 'navigate') return caches.match('index.html');
      });
    })
  );
});
