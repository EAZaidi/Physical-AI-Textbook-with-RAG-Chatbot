// Link checker for Module 2 external URLs
const https = require('https');
const http = require('http');

const urls = [
  'https://www.docker.com/products/docker-desktop/',
  'https://www.xquartz.org/',
  'https://sourceforge.net/projects/vcxsrv/',
  'https://gazebosim.org/docs/fortress/',
  'http://sdformat.org/spec',
  'https://ode.org/wiki/index.php/Manual',
  'https://github.com/ros-simulation/gazebo_ros_pkgs',
  'https://en.wikipedia.org/wiki/List_of_moments_of_inertia',
  'https://unity.com/download',
  'https://github.com/Unity-Technologies/Unity-Robotics-Hub',
  'https://github.com/Unity-Technologies/URDF-Importer/blob/main/Documentation~/UrdfImporter.md',
  'https://docs.unity3d.com/2022.3/Documentation/ScriptReference/ArticulationBody.html',
  'https://docs.unity3d.com/Manual/PhysicsOverview.html',
  'https://github.com/Unity-Technologies/ROS-TCP-Connector',
  'https://docs.unity3d.com/Manual/Profiler.html',
  'https://gazebosim.org/api/sensors/6.0/index.html',
  'https://github.com/ros2/common_interfaces/tree/rolling/sensor_msgs',
  'http://www.open3d.org/docs/release/',
  'https://pointclouds.org/',
  'https://ieeexplore.ieee.org/document/6696917',
  'https://docs.ros.org/en/humble/',
  'https://www.docker.com/blog/robotics/',
  'https://community.gazebosim.org/'
];

async function checkUrl(url) {
  return new Promise((resolve) => {
    const protocol = url.startsWith('https') ? https : http;
    const timeout = 5000;

    const req = protocol.get(url, { timeout }, (res) => {
      if (res.statusCode >= 200 && res.statusCode < 400) {
        resolve({ url, status: 'OK', code: res.statusCode });
      } else if (res.statusCode >= 300 && res.statusCode < 400) {
        resolve({ url, status: 'REDIRECT', code: res.statusCode, location: res.headers.location });
      } else {
        resolve({ url, status: 'ERROR', code: res.statusCode });
      }
      req.abort();
    });

    req.on('error', (err) => {
      resolve({ url, status: 'FAILED', error: err.message });
    });

    req.on('timeout', () => {
      req.abort();
      resolve({ url, status: 'TIMEOUT' });
    });
  });
}

async function checkAllLinks() {
  console.log(`Checking ${urls.length} external links in Module 2...\n`);

  const results = await Promise.all(urls.map(checkUrl));

  const ok = results.filter(r => r.status === 'OK');
  const redirects = results.filter(r => r.status === 'REDIRECT');
  const errors = results.filter(r => r.status === 'ERROR' || r.status === 'FAILED' || r.status === 'TIMEOUT');

  console.log('=== RESULTS ===\n');
  console.log(`✅ OK: ${ok.length}`);
  console.log(`➡️  REDIRECTS: ${redirects.length}`);
  console.log(`❌ ERRORS: ${errors.length}`);

  if (errors.length > 0) {
    console.log('\n=== BROKEN LINKS ===');
    errors.forEach(r => {
      console.log(`❌ ${r.url}`);
      console.log(`   Status: ${r.status} ${r.code || ''} ${r.error || ''}\n`);
    });
  }

  if (redirects.length > 0) {
    console.log('\n=== REDIRECTS (May need updating) ===');
    redirects.forEach(r => {
      console.log(`➡️  ${r.url}`);
      console.log(`   → ${r.location}\n`);
    });
  }

  console.log('\n=== SUMMARY ===');
  console.log(`Total links checked: ${urls.length}`);
  console.log(`Working links: ${ok.length + redirects.length}`);
  console.log(`Broken links: ${errors.length}`);

  if (errors.length === 0) {
    console.log('\n✅ All external links in Module 2 are working!');
    process.exit(0);
  } else {
    console.log('\n⚠️  Some links need attention.');
    process.exit(1);
  }
}

checkAllLinks();
