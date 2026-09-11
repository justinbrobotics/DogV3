# Third-party software notices

`dogv3/runtime/static/three.min.js` is vendored Three.js browser code. Its embedded MIT attribution is retained; the upstream license text is reproduced below. The asset is used by the operator interface's 3D view.

Source project: https://github.com/mrdoob/three.js

License source: https://raw.githubusercontent.com/mrdoob/three.js/r159/LICENSE

## Three.js — MIT License

Copyright © 2010-2023 three.js authors

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

## Installed dependencies

Python packages are declared in `pyproject.toml`; PlatformIO obtains the board framework specified in `firmware/dogv3_mux/platformio.ini`. Pi system packages are listed in `deploy/pi/install.sh`. They are fetched from their package sources at installation time and retain their own license notices. No installed interpreter, environment, framework cache or dependency binary is bundled in this folder.
