Vendored Webots assets
======================

This directory mirrors the 'projects/' tree of the Webots repository
(https://github.com/cyberbotics/webots, tag R2025a) for every PROTO, texture and mesh
that the worlds of this project reference. The world files and these PROTO files reference
each other with RELATIVE paths, so Webots never needs to download anything from GitHub.

Provenance of every file: MANIFEST.tsv (local path, source URL, sha1 of the local file,
bytes, sha1 of the upstream bytes before relative-path rewriting when recorded; PROTO files differ
from upstream only in their EXTERNPROTO/url string literals).
Licences: Cyberbotics assets are 'Licensed for use only with Webots'
(https://cyberbotics.com/webots_assets_license); PAL Robotics TIAGo files are Apache-2.0;
see the header comments of each PROTO file.

Regenerate / extend: python3 tools/vendor_webots_assets.py worlds/<world>.wbt
