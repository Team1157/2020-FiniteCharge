#Read Me
This is FRC team 1157's code for the 2020 First Robotics Competition. To contribute, read below.
##Setting up your development environment
###1. Intellij
We are using Jetbrains' Intellij for our development, and we recommend that you do as well.
You can download the free community edition [here](https://www.jetbrains.com/idea/download/).
You can also request a license for the ultimate edition as a student, but it won't be necessary for this project.

###2. WPILib
Once you have installed Intellij, you will need to install wpilib.
This is a free software library for first teams required for development.
You can download it [here](https://github.com/wpilibsuite/allwpilib/releases/tag/v2020.1.2).
Scroll down to "Assets", and choose whichever release matches your operating system.
Once downloaded, you should extract and run the installer.

###3. Git
Git is a version management tool. If you already have it installed, you can skip this step.
If not, you can download it [here](https://git-scm.com/download/).
Click on the appropriate operating system, then run the installer once it downloads.
You can leave everything in the installer on its default settings.

##Downloading the codebase
Once you've set up your environment, you can download and edit our code.
Launch Intellij, and on the main menu, choose "Check out from Version Control".
In the drop down menu, choose "Git".
In the URL field, paste this repositiory's url: 
https://github.com/Team1157/2020-InfiniteRecharge.

Next, select a location for your local files, whatever you prefer. Finally, click "Clone".
You will be asked to provide your Github username and password.
Intellij will automatically download the repository's files.

##Contributing code
Once you've downloaded our code, you can begin editting and uploading your own contributions.
Whenever you open the project, you should update your local copy by clicking the blue arrow in the upper right of the Intellij window, or by pressing `Control+T`.
Then, you can start editing or adding code.

By default, the changes you make are only present in your copy of the code.
If you want the rest of the team to be able to see and use your code, you will need to commit it.
To commit your changes, click the green checkmark in the upper right of the Intellij window, or press `Control+K`.

You will be asked to write a commit message.
Commit message should be short and informative, and they should describe the changes you made. For example: "Added setpoints to lift subsystem" or "Added vision pipeline".
When you're done, click commit. Finally, you need to push that commit to the central repository,
which can be accomplished via <b>VCS | Git | Push</b> or `Control+Shift+K`.
Alternatively you can commit and push at the same time using `Control+Alt+K`.

##Style guidelines
When writing code, please make sure other can easily read and understand it.
Include comments whenever you write a new class, method, or function, and whenever you add more than a few lines of code to an existing block.
