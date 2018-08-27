v3.0.7
------
- Updated Slang to 0.10.31

Bug Fixes:
- Fixed a crash when rendering a VariablesBuffer/ConstantBuffer UI without specifying a group name

v3.0.6
------
- Changed max bones to 256

Bug Fixes:
- Updated Slang to 0.10.30. Fixes SceneEditor shaders in Vulkan configs
- Apply scaling transforms in animations
- Fixed interpolation issues at the end of animations

v3.0.5
------
- Added support for exporting BMP and TGA images.
- Added `ConstantBuffer::renderUI()` to automatically render UI for editing a constant buffer's values.

Bug Fixes:
- Fixed crash when setting ForwardRenderer sample to MSAA with sample count 1
- std::string version of Gui::addTextBox() now correctly updates the user's string
- Fixed row-pitch calculation when copying texture subresources in DX12

v3.0.4
------
- Updated Slang to 0.10.24
- Added an option to create a `Program` from a string
- Added `CopyContext::updateSubresourceData()` which allows updating a region of a subresource
- Added `Program::Desc` has a new function - `setShaderModel()`. It allows the user to request shader-model 6.x, which will use dxcompiler instead of FXC
- Added support for double-quotes when parsing command line arguments. Text surrounded by double-quotes will be considered a single argument.

v3.0.3
------
- Added FXAA as an effect
- Support programmable sample position - `Fbo::setSamplePositions()` (DX only)
- Added RenderContext::resolveResource() and RenderContext::resolveSubresource() MSAA resolve functions
- Added support for setting shading model through fscene files and load flags. Also editable in Scene Editor

v3.0.2
------
- Various bug fixes
- Fixed Vulkan error spam seen when running Falcor's included samples
- Updated API abstraction interfaces to return const-ref where applicable
- Fixed crash when handling mouse/keyboard messages after the renderer has shut down

v3.0.1
------
- Added RenderContext::StateBindFlags, which allows the user to control which part of the `GraphicsState` will be bound to the pipeline
- Added helper functions to initialize D3D12 state-objects descs (root-signature, GSO-desc)
- Added a function that creates a texture from an resource API handle
- Added a new sample: LightProbeViewer. Allows you to see how light probe images look after pre-integration with various sample counts and materials.
