# RUBIK CUBE COLOR RECOGNITION

## GENERAL PROCESS
- The purpose of this python script is to detect the colors of each face of the Rubik's cube, so that the robot can decide what the sequence of moves is going to be to solve it.

- We import as an image(e.g. ***.jpeg***) all the faces of the cube. On the photo we create a 3x3 grid to detect all the 9 stickers of the cube.

- We process those images to extract and analyze color data. We crop the center of an image, divide it into a 3x3 grid, and sample the median color of each cell. The color values are converted to the CIELAB color space (ideal for robust color recognition under varying lighting conditions) and plotted into a 3D scatter plot using their actual RGB colors.

## CUBES ROTATIONS & SCANNING PROCESS

The robot reads each face of the cube, starting by picking the cube with its left hand. 
We scan the first three faces of the cube with the right hand, and then we continue the process with the left hand. 

1. RIGHT HAND SEQUENCE 
- The process begins with the **Front face**.
- The gripper of the robot is rotated 90° to the right so it can read the **left face**. 
- For the **back face** we have to bring the hand back to its initial position (90° to the right and rotate the cube 180°).

2. LEFT HAND SEQUENCE
- The left hand takes control to scan the remaining faces.
- With the left hand we pick the cube, and we rotate it 90° to scan the **top face**
- Next a 90° rotation is made to the left to scan the **right face**
- Finally, we flip the cube by 180° to scan the **bottom face**.

The diagram below is how we see the cube when we unwrap it, following the sequence of the motions that we are using.

```
       +-------+
       | FRONT |
+------+-------+-------+------+
| LEFT |  UP   | RIGHT | DOWN |
+------+-------+-------+------+
       | BACK  |
       +-------+
```

### NOTE 
We have to keep in mind, that when we scan each face, most of them are rotated. So when we create the final string with all the colors detected, we have to rotate them according to the side. 

For example, the back side is 180° rotated. So we have to rotate the colors upside down.
