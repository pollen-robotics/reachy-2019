# Contributing to Reachy

:+1::tada: First off, thanks for taking the time to contribute! :tada::+1:

The following is a set of guidelines for contributing to Pollen community and its packages, which are hosted in the [Pollen Robotics organization](https://github.com/pollen-robotics) on GitHub. These are mostly guidelines, not rules. Use your best judgment, and feel free to propose changes to this document in a pull request.

## Code of Conduct

This project and everyone participating in it is governed by the [Code of Conduct](CODE_OF_CONDUCT.md). By participating, you are expected to uphold this code. Please report unacceptable behavior to [contact@pollen-robotics.com](mailto:contact@pollen-robotics.com).

## I don't want to read this whole thing I just have a question!!!

Our forum is probably the right place to do that. It's where the community chimes in with helpful advice if you have questions.

* [Pollen Robotics forum](https://forum.pollen-robotics.com)

## What should I know before I get started?

### Reachy and Playgrounds

Reachy is a large open source project. You will find the code related to the robot itself in [this repository](https://github.com/pollen-robotics/reachy-2.0). The code/hardware/data related to a specific environment or playground will be directly in its separate repository (e.g. [TicTacToe environment](https://github.com/pollen-robotics/reachy-tictactoe)).

## How Can I Contribute?

### Reporting Bugs

This section guides you through submitting a bug report for Atom. Following these guidelines helps maintainers and the community understand your report :pencil:, reproduce the behavior :computer: :computer:, and find related reports :mag_right:.

> **Note:** If you find a **Closed** issue that seems like it is the same thing that you're experiencing, open a new issue and include a link to the original issue in the body of your new one.

#### Before Submitting A Bug Report

* **Check the [FAQs on the forum](https://discuss.atom.io/c/faq)** for a list of common questions and problems.
* **Determine [which repository the problem should be reported in](#reachy-and-playgrounds)**.
* **Perform a [cursory search](https://github.com/search?q=is%3Aissue+user%3Apollen-robotics)** to see if the problem has already been reported. If it has **and the issue is still open**, add a comment to the existing issue instead of opening a new one.

#### How Do I Submit A (Good) Bug Report?

Bugs are tracked as [GitHub issues](https://guides.github.com/features/issues/). After you've determined [which repository](#reachy-and-playgrounds) your bug is related to, create an issue on that repository and provide the following information by filling in [the template](https://github.com/pollen-robotics/reachy/.github/blob/master/.github/ISSUE_TEMPLATE.md).

Explain the problem and include additional details to help maintainers reproduce the problem:

* **Use a clear and descriptive title** for the issue to identify the problem.
* **Describe the behavior you observed after following the steps** and point out what exactly is the problem with that behavior.
* **Explain which behavior you expected to see instead and why.**

### Pull Requests

The process described here has several goals:

- Maintain Reachy's quality
- Fix problems that are important to users
- Engage the community in working toward the best possible Reachy
- Enable a sustainable system for Reachy's maintainers to review contributions

Please follow these steps to have your contribution considered by the maintainers:

1. Follow all instructions in [the template](PULL_REQUEST_TEMPLATE.md)
2. Follow the [styleguides](#styleguides)
3. After you submit your pull request, verify that all [status checks](https://help.github.com/articles/about-status-checks/) are passing <details><summary>What if the status checks are failing?</summary>If a status check is failing, and you believe that the failure is unrelated to your change, please leave a comment on the pull request explaining why you believe the failure is unrelated. A maintainer will re-run the status check for you. If we conclude that the failure was a false positive, then we will open an issue to track that problem with our status check suite.</details>

While the prerequisites above must be satisfied prior to having your pull request reviewed, the reviewer(s) may ask you to complete additional design work, tests, or other changes before your pull request can be ultimately accepted.

## Styleguides

Start reading our code and you'll get the hang of it. We optimize for readability:

We use [flake8](https://flake8.pycqa.org/en/latest/) for Python linting with very [minimal customisation](https://github.com/pollen-robotics/reachy-2.0/blob/master/software/setup.cfg).
Please make sure your contribution fit with this standard before submitting it. It will be automatically checked during CI.



