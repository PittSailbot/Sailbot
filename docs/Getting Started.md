This guide walks you through all of the steps to get started with contributing to Sailbot! :D

## Installing Git & Using Github
Git is the industry standard for collaborating on projects. It allows multiple people to work on different features and easily share changes. If you're majoring in CS you'll be forced to learn this eventually, and its a great time to get a head start for your classes.

1\. Install [Git](https://git-scm.com/downloads)

> Don't know git? Learn [here](https://medium.com/free-code-camp/learn-the-basics-of-git-in-under-10-minutes-da548267cc91). 

> Need a refresher? Here's a [cheat sheet](https://education.github.com/git-cheat-sheet-education.pdf). 

2\. Navigate over to [Github](https://github.com/PittSailbot/Sailbot). If you don't have an account, create one! Using Github early on is great for saving progress and safeguarding against academic dishonesty claims (hopefully *very rare*)

## Installing Python
Pretty much everything we write is using Python. Its much easier to pick up than other languages and has good supporting libraries for working with the Raspberry Pi.

3\. Install [Python 3.10](https://www.python.org/downloads/release/python-31011/)

- Downloads are at the bottom of the page. You'll want to download either Windows 64-bit or macOS.


> Never used Python before? We're open to help! If you're taking CS 0010 or CMPINF 0010 you'll get to learn in your classes. If not, here's some [resources](https://automatetheboringstuff.com/) to learn some of the basics if you're willing to put in the time.

> Or, if you just need a refresher or want to see some *cool advanced features* [click here](https://gto76.github.io/python-cheatsheet/).

## Installing IDE/Code Editor
Pycharm is a great editor for Python with lots of advanced features. VS Code is also another great option if you'd prefer a lightweight editor that works with any language.

4\. Download [Pycharm](https://www.jetbrains.com/pycharm/download/#section=windows)

- You can get the professional edition for free if you sign up with your university email.

5\. Install Pycharm

- In the Installation Options, you'll want to tick these options.
	![](\assets\Pycharm Installation Options.png)

6\. Launch Pycharm and select "Get from VCS" in the welcome menu

> Pycharm keeps all projects in the /PycharmProjects folder. If you want to install the repo somewhere else, then navigate to where you want to install the repository in the terminal using `cd C:\Path/to/folder`.

7\. Paste this URL and click 'clone'
```console
https://github.com/PittSailbot/Sailbot.git
```
8\. After the Sailbot repo opens, in the bottom right click where it says 'no interpreter'

9\. Click add new interpreter>add local interpreter

10\. Under 'Base interpreter', select Python 3.10 and tick 'Inherit global site-packages'

> If you think the Pycharm editor is a lil fugly or overwhelming try going to File>Settings>Appearance & Behavior>New UI and tick the 'Enable new UI' box. You can also download themes which is a disastrous rabbit hole. My favorite is Monokai Pro. :)

Pycharm has a pretty nifty integrated terminal. There's dedicated shortcuts and icons to access everything but for now lets use the "Search Anywhere" feature which is **super** helpful for finding what you need.

11\. Double click shift, search 'terminal' and hit enter

12\. In the integrated terminal type:
```console
pip install -r requirements-dev.txt
```

This will install most of the third party modules that we need. There may be some modules we've missed which you can install using pip. Some modules like the adafruit libraries and GPIO can only be installed on the Raspberry Pi which is ok.

## Testing our Environment

We use pytest to ensure that our code works (most of the time). 

13\. In the integrated terminal enter:
```console
python3 -m pytest test_all.py
```
This will run all of the tests we've done. Some tests will get skipped because we're not running our code on the pi. If the tests run correctly then congrats, you're ready to contribute! If there's any errors showing call over either Aaron, Tom or Jonah.
