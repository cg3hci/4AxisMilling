# Automatic Surface Segmentation for Seamless Fabrication Using 4-axis Milling Machines

Stefano Nuvoli, Alessandro Tola, [Alessandro Muntoni](http://vcg.isti.cnr.it/~muntoni/), [Nico Pietroni](https://profiles.uts.edu.au/Nico.Pietroni), [Enrico Gobbetti](https://www.crs4.it/it/peopledetails/8/enrico-gobbetti/), [Riccardo Scateni](http://people.unica.it/riccardoscateni/)<br/>
*EuroGraphics 2021*<br/>

![alt text](misc/teaser.png)

## Abstract
We introduce a novel geometry-processing pipeline to guide the fabrication of complex shapes from a single block of material using 4-axis CNC milling machines. This setup extends classical 3-axis CNC machining with an extra degree of freedom to rotate the object around a fixed axis. The first step of our pipeline identifies the rotation axis that maximizes the overall fabrication accuracy. Then we identify two height-field regions at the rotation axis’s extremes used to secure the block on the rotation tool. We segment the remaining portion of the mesh into a set of height-fields whose principal directions are orthogonal to the rotation axis. The segmentation balances the approximation quality, the boundary smoothness, and the total number of patches. Additionally, the segmentation process takes into account the object’s geometric features, as well as saliency information. The output is a set of meshes ready to be processed by off-the-shelf software for the 3-axis tool-path generation. We present several results to demonstrate the quality and efficiency of our approach to a range of inputs

## Source Code
Source code is hosted on this GitHub repository.

### Download
```bash
git clone --recursive https://github.com/cg3hci/4AxisMilling
```
### Build

```
mkdir build
cd build
cmake ..
```

### Run

```
./fourAxisMilling -i=input_mesh.obj -o=output_directory [parameters]
```


## License
[MPL2](LICENSE) licensed
([FAQ](https://www.gnu.org/licenses/gpl-faq.html))



