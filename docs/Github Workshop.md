## Topics

* version control, conceptually
* branching
* committing & pushing to GitHub
* undoing changes

# Version Control with `git` and GitHub

As code became, more and more, a large commercial endeavour with many contributors across different teams, cities, and timezones, the process of *tracking versions* of software became non-trivial.

## Enter the VCS

A **version control system**, or VCS, tracks the history of changes as people and teams collaborate on projects together. As the project evolves, teams can run tests, fix bugs, and contribute new code with the confidence that any version can be recovered at any time. Developers can review project history to find out:

* Which changes were made?
* Who made the changes?
* When were the changes made?
* Why were changes needed?

## Diversion: what actually *is* a repository?

A **repository** is usually used to organize a single project. Repositories can contain folders and files, images, videos, spreadsheets, and data sets – anything your project needs. GitHub makes it easy to add a README and other files, such as licenses and codes of conduct. 

So, you know how you can use `ls` with the `-a` flag to print *all* the files, even the hidden ones? Let's do that here; it'll reveal something cool. If you do `ls -a`, you'll see two files you didn't know were there: `.git` and `.gitignore`.

`.gitignore` is a file where you can specify files for `git` to, well, ignore. Things like sensitive passwords or databases, you don't want to put up on GitHub for everyone to see, so this lets you quickly tell `git` to not save anything you don't want it to.

`.git` is much deeper, though: it's where the repository actually lives. `.git` is actually a *directory* which contains information about the repository and a list of every change you've ever made in the repository. Every time you tell `git` to save your work, it'll track everything you've altered, removed, or added to the repository in that `.git` folder. That way, if you ever want to revert back to a previous change, you can do so. And, additionally, anyone who can see your repository can look at how your code has changed and developed over time.

# Git Workshop

If git isn't already installed, install it [here](https://git-scm.com/downloads)! Make sure you have a [Github account](https://github.com/) too!

We should have the Sailbot repo already set up. If not, follow the steps in [Getting Started](Getting Started.md#Installing IDE/Code Editor). Open the project in Pycharm.

## Making our branch

**Branches** in `git` are versions of your project that you separate from the original branch, which is called `main`. Ours is called `main` because we made the repository on GitHub. But, if you make a repo through `git` itself, the original branch will be called `master`. You might hear someone refer to an original branch as `master`, so just remember that they both sorta refer to the same thing.

We're going to be making our own branch to work on changes. Within Pycharm, click on the Sailbot>main tab at the top right (its got a lil tree next to it). Click new branch and name it after yourself or preferably the issue you're working on (if applicable). 

> Alternatively, in the terminal, you can create a new branch with the command `git checkout`, using the `-b` flag, which (you guessed it) stands for "branch".

Note: if git asked you to login with your GitHub username and a _token_, check out [this guide on making a token](https://help.github.com/en/github/authenticating-to-github/creating-a-personal-access-token-for-the-command-line). If you do need to do so, the only box you need to check is `repo`; you can ignore the rest.

Lets confirm that our branch was created. Go to the [Sailbot repo](https://github.com/PittSailbot/Sailbot/branches) and check to make sure that your branch was created. If so, great! If not, let me know.

## Lets mess around

Take a look around our repository. *Pretty nifty right?* Lots of the files in /Sailbot are config files and boilerplate for ROS and automated testing. We won't have to worry about any of that right now.

For now, lets cause some chaos. Add any files you want or make random changes. We'll be undoing all of these later so don't worry!

## Commiting your work

You tell `git` to save your work by using the command `git commit`. Committing is *like* hitting "Save" on a Word document, but it's a little more intentional than that; because you have to specifically tell `git` why you're saving, a **commit** is a human-meaningful amount of work. 

At the top left of Pycharm look for a lil -o- symbol. This tracks all of our local changes. Pycharm has a cool feature to group specific changes into Changelists. This is especially helpful when working on multiple parts of the codebase at once. Instead of publishing *all* of our changes, we can cherry pick which changes we want to push and have relevant commit messages for each.

> Alternatively, in the terminal run `git add .` to add any untracked files and `git commit -m "my first commit"`. The `-m` flag stands for **message**, and that's how you tell `git` and, by extension, anyone who looks at your repository, what you did for this commit. 


## Publishing our work

Now that we've made all the changes we need, lets push them to our branch. Make sure you're in your own branch for this (but it won't nuke anything if you aren't). 

In Pycharm's commit tab, checkmark all of the files you'd like to commit. At the bottom write in your commit message. In real commits, try to summarize all of the changes you'd made with a bulleted list.

After writing your commit message, hit commit and publish. 

> Alternatively, in the terminal run `git push`


## Oh shit! Where's the undo?

No one is immune to the "oh shit" moments that happen when pushing code. Sometimes we catch a bug the second it goes live, sometimes we accidentally nuke the repo. It happens. But unlike numbering and saving versions of your code locally. 99% of the time we can recover. That's what git is for.

In Pycharm, hit the little tree in the bottom left corner. This will show our entire version history. Right now, you should see the commit you just pushed at the top of the list.

There are many ways we can revert this progress. In Pycharm, there are three main ways we can 'undo' this commit. 
- `undo` (only works on non pushed commits) will unstage the files and return the changes to your local copy.
- `revert` will make create another commit, undoing the changes you've made. 
- `drop` will delete the commit from the git history like it was never pushed in the first place.

For now lets drop the commit. Our branch should be identical to main now.
