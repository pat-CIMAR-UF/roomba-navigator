# roomba-navigator

This is a repository about ROS1 code for controlling an iRobot Roomba with an Xbox One controller, to practice robotics related applications such as SLAM (Simultaneous Localization And Mapping) and path planning.

# Install

```bash
$ cd catkin_ws/src
$ git clone --recursive https://gitlab.com/roomba-bot/roomba-navigator.git .
```
The `--recursive` is to download all the dependent packages such as [scanse-driver](https://github.com/scanse/sweep-ros) in this repository.
The `.` in the end of the command line is important, since it stops creating another folder when you use `git clone` command.

After downloading everything, make the package:
```bash
$ cd catkin_ws
$ catkin_make
```

# Collaboration Rule

To collaborate in this repository, please follow instructions below:

+ Create your local branch:
    ```bash
    $ cd catkin_ws/src
    $ git checkout -b local_branch
    ```
+ Make changes: `git add` and `git commit` 
    + Please **comment** in each commit to let others know what contributions you made. This **[commit style](http://udacity.github.io/git-styleguide/)** is strongly recommended.
+ Push to the repository:
    ```bash
    $ git push origin local_branch
    ```
+ Then create a [merge request](https://docs.gitlab.com/ee/gitlab-basics/add-merge-request.html) to master branch





