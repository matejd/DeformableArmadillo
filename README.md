Deformable Armadillo
====================

An experiment in position-based dynamics. The idea was to simulate a deformable object, not necessarily in a completely
physically-rigorous way, but at least in a physically-plausible way. Online [here](http://matejd.github.io/DeformableArmadillo/bin/emscripten/DeformableArmadillo.html).

![Screenshot](http://matejd.github.io/DeformableArmadillo/docs/armadillo-screenshot1.png)


Build
-----

No fancy build systems, just run `make linux` or `make emscripten`.
Building for Linux requires glfw, glew, AntTweakBar and optionally Assimp and CGAL.
glm, cereal and AntTweakBar headers are included in `framework/external`.

Emscripten 1.26.0 requires a tiny patch (see [issue](https://github.com/kripken/emscripten/issues/2949)),
changing line [490](https://github.com/kripken/emscripten/blob/master/src/library_glfw.js#L490) from
`win.mousePosFunc = cbfun;` to `win.cursorPosFunc = cbfun;`. Also, for reasons unknown, I had
to specify full path to Emscripten's `libcxxapi/include`. The path in makefile will need to be modified.
See makefile for more details.


References and resources
------------------------

Müller, M., Heidelberger, B., Hennix, M., & Ratcliff, J. (2006). Position Based Dynamics.

Bender, J., Müller, M., Otaduy, M. A., Teschner, M., & Macklin, M. (n.d.). A Survey on Position-Based Simulation Methods in Computer Graphics, M.

Bender, J., Koschier, D., Charrier, P., & Weber, D. (2014). Position-Based Simulation of Continuous Materials.

Matthias, M., & Gross, M. (n.d.). Interactive Virtual Materials.

Irving, G., Teran, J., & Fedkiw, R. (2004). Invertible Finite Elements For Robust Simulation of Large Deformation.

FEM Simulation of 3D Deformable Solids (SIGGRAPH 2012 Course): http://femdefo.org/

Wojtan, C. (2010). Animating Physical Phenomena with Embedded Surface Meshes

Tom Forsyth's Linear-Speed Vertex Cache Optimisation algorithm (implementation by [Martin Storsjo](https://github.com/vivkin/forsyth))

Glenn Fiedler. Fix your timestep! http://gafferongames.com/game-physics/fix-your-timestep/

Compact Normal Storage: http://aras-p.info/texts/CompactNormalStorage.html

Stanford Armadillo: http://graphics.stanford.edu/data/3Dscanrep/
(decimated with MeshLab: http://meshlab.sourceforge.net/)
