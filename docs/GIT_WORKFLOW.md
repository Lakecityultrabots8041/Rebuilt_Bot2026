# Git, GitHub & CI/CD - How We Work Together

Chris Shaw - Lake City Ultrabots Programming Mentor
2026 REBUILT season

This is how Team 8041 writes code together without stepping on each other or losing work. Every programmer on the team reads this once and keeps it handy. It is not optional - the rules here are what stop us from overwriting each other's code the night before a competition.

If you only remember one thing: **never work directly on `main`, and always pull before you start.**

---

## The problem this solves

When two people edit the robot code and one of them pushes over the other, work disappears. It usually happens like this:

1. Two students both start from an old copy of the code.
2. One finishes and pushes.
3. The other finishes, gets a conflict, panics, and force-pushes or "takes their version" - erasing the first person's work.
4. At competition, a mentor is left hand-merging branches and guessing what was lost.

Git is built to make this *impossible* if you use it correctly. The rest of this doc is the correct way.

---

## The golden rules (read these every time)

1. **`main` is protected. Nobody pushes to `main` directly.** All changes go through a Pull Request (PR).
2. **One branch per task.** Start it from a fresh, up-to-date `main`.
3. **Pull `main` before you start, and again before you open a PR.** Stale branches cause conflicts.
4. **Small PRs.** One feature or one fix. A 40-line PR gets reviewed in minutes; a 2,000-line PR sits for days and hides bugs.
5. **Never force-push to a shared branch.** `git push --force` is how code gets destroyed. If you think you need it, ask a mentor first.
6. **If you are unsure, stop and ask.** A two-minute question beats a lost night of work.

---

## One-time setup (do this once per laptop)

### 1. Tell Git who you are

This is how your commits get attributed to you. Use your real name and the email tied to your GitHub account.

```bash
git config --global user.name "Your Name"
git config --global user.email "you@example.com"
```

Check it worked:

```bash
git config --global user.name
git config --global user.email
```

If commits show up as the wrong person on GitHub, this is almost always the cause.

### 2. Clone the repo

```bash
git clone https://github.com/Lakecityultrabots8041/Rebuilt_Bot2026.git
```

Open the folder in **WPILib VS Code** (not regular VS Code - the WPILib version has the robot tools built in).

### 3. Make sure it builds before you change anything

```bash
./gradlew build
```

On Windows you can use `gradlew.bat build` if the first form gives trouble. If it builds clean, you have a good starting point. If it does not build *before* you touched anything, tell a mentor - do not start layering changes on top of a broken build.

---

## The branch model

```
main  ──●──────●──────●──────●───────────►   (protected, always builds, competition-ready)
         \            /      /
          ●──●──●────●      /                 feature branch: feat/intake-tuning
                          /
                  ●──●──●                     feature branch: fix/led-id
```

- **`main`** is the single source of truth. It must always build and should always be safe to deploy. You cannot push to it directly - that is enforced on GitHub (see [Branch protection](#branch-protection-mentor-setup)).
- **Feature branches** are where all real work happens. Each one is short-lived: you branch off `main`, do one task, open a PR, get it merged, and delete the branch.

### Branch naming

Use a prefix and a short description with dashes:

| Prefix     | Use it for                          | Example                    |
|------------|-------------------------------------|----------------------------|
| `feat/`    | A new feature                       | `feat/climber-control`     |
| `fix/`     | A bug fix                           | `fix/shooter-stall`        |
| `tune/`    | Tuning constants/PID only           | `tune/drive-pid`           |
| `docs/`    | Documentation only                  | `docs/vision-update`       |
| `auto/`    | Autonomous paths/routines           | `auto/center-3-piece`      |

A good branch name tells everyone what you are working on before they even open it. That alone prevents two people from grabbing the same task.

---

## The daily workflow (the loop you repeat all season)

### Step 1 - Get the latest `main`

```bash
git checkout main
git pull
```

This pulls everyone else's merged work onto your machine. **Do this every single time before starting.** Skipping it is the number one cause of conflicts.

### Step 2 - Make a branch for your task

```bash
git checkout -b feat/intake-tuning
```

Now you are on your own branch. Anything you do here is isolated from `main` and from everyone else.

### Step 3 - Do your work, committing as you go

Make a change, build it, then commit it. Commit in small logical chunks, not one giant commit at the end.

```bash
git add .
git commit -m "Raise intake supply current limit to 80A"
```

(See [Writing good commits](#writing-good-commits) for what makes a good message.)

### Step 4 - Push your branch to GitHub

The first time you push a new branch:

```bash
git push -u origin feat/intake-tuning
```

After that, just `git push`. Push often - a pushed branch is backed up on GitHub. A branch that only lives on your laptop is one spilled drink away from gone.

### Step 5 - Catch up with `main` before opening your PR

While you were working, others may have merged things. Pull their work into your branch so you handle any conflicts now, on your machine, instead of dumping them on the reviewer:

```bash
git checkout main
git pull
git checkout feat/intake-tuning
git merge main
```

If there are conflicts, resolve them carefully (see [Resolving conflicts without losing code](#resolving-conflicts-without-losing-code)), then build again to confirm everything still works.

### Step 6 - Open a Pull Request

See [How to open a PR](#how-to-open-a-pr).

### Step 7 - After it merges, clean up

Once your PR is merged, delete the branch (GitHub offers a button) and start your next task from a fresh `main`:

```bash
git checkout main
git pull
```

---

## Writing good commits

A commit is a labeled snapshot. Months from now, the commit history is how we figure out *when* and *why* something changed. Lazy messages make that impossible.

### Message format

- **First line: a short summary, under ~70 characters, in the present tense.** "Add climber hold command", not "added stuff" or "fixes".
- **Blank line, then details** if the change needs explaining - especially *why*, not just *what*.

Good:

```
Lower flywheel stator limit to 80A

Phoenix 6 default was 120A, which tripped the breaker during
back-to-back shots. 80A holds RPM without browning out the bus.
```

Bad:

```
stuff
```

```
asdf fixed it
```

### Why the "why" matters

The diff already shows *what* changed. It cannot show *why*. "Lower flywheel stator limit to 80A" plus the reason tells the next person not to bump it back up without understanding the tradeoff. This is the same habit we use in our other docs - decisions come with reasons.

### Signing your work: co-authors

When two people pair on a change, credit both. Add a `Co-Authored-By` trailer at the **bottom** of the commit message, after a blank line:

```
Add vision-based shooter speed table

Co-Authored-By: Jane Student <jane@example.com>
```

The email must match the co-author's GitHub email for GitHub to link them. You can add more than one trailer line for more than one co-author. This is how everyone who did the work gets credit, not just whoever happened to type the commit.

> We do not require cryptographic (GPG/SSH) commit signing on this team - it is real setup on every laptop. Setting your `user.name`/`user.email` correctly and using co-author trailers is what we expect. If you want the green "Verified" badge on your commits, ask a mentor and we will help you set it up.

### Do not commit generated or junk files

- `TunerConstants.java` is generated by CTRE Tuner X. Regenerate it from the tool; do not hand-edit it.
- Build output (`build/`, `.gradle/`) and editor settings are ignored on purpose - see [`.gitignore`](../.gitignore). If `git status` shows hundreds of files, you are probably about to commit something you should not. Stop and check.

---

## Working together without collisions

Most "lost code" is really two people editing the same thing at the same time. You avoid that with communication and small scope, not with Git tricks.

1. **Call your shot.** Before you start, say what you are taking (in person or in the team chat) and name your branch to match. If someone already has `feat/intake-tuning` going, coordinate instead of opening a competing branch.
2. **Own subsystems loosely.** Try to have one person own a subsystem at a time (intake, shooter, vision...). Two people in the same file at once is the highest-risk situation. The code is split into `subsystems/` folders precisely so people can work in parallel - stay in your lane when you can.
3. **Keep PRs small and frequent.** Merge a small working piece today rather than a giant branch next week. The longer a branch lives, the more `main` drifts away from it and the worse the eventual merge.
4. **Pull often.** Even mid-task, `git checkout main && git pull && git checkout your-branch && git merge main` keeps you close to everyone else so conflicts stay tiny.

---

## Handling complex / large additions

Some work is genuinely big - a whole climber subsystem, a new autonomous framework. Do not try to land it in one massive PR. That is exactly the kind of change that causes painful merges.

- **Break it into stages.** Land the constants and an empty subsystem first, then the basic commands, then tuning. Each stage is its own small PR that builds on the last. The robot stays working the whole way.
- **Use a Draft PR for work in progress.** Open the PR early and mark it **Draft** (GitHub option when you create it). This shows everyone what you are building, lets CI run on it, and lets a mentor look early - without anyone merging it before it is ready. Mark it "Ready for review" when it is done.
- **For a long multi-person effort, use a shared integration branch.** Instead of everyone targeting `main`, agree on one branch like `feat/climber` and open smaller PRs into *that*. When the whole feature is done and tested, one PR brings `feat/climber` into `main`. Ask a mentor to set this up - it is more overhead, so we only do it for truly big features.
- **Keep it in sync.** Whatever the approach, merge `main` into your long-lived branch regularly so it never drifts far.

---

## How to open a PR

1. Push your branch (Step 4 above).
2. Go to the repo on GitHub. It will show a banner: **"Compare & pull request"** - click it. (Or go to the **Pull requests** tab and click **New pull request**.)
3. Set the **base** to `main` and the **compare** to your branch.
4. The [PR template](../.github/pull_request_template.md) fills in automatically. Fill out every section - what it does, what you tested, the checklist.
5. If it is not finished, click the dropdown on the green button and choose **Create draft pull request**.
6. Request a review from a mentor (or whoever owns that subsystem).

A PR with a filled-out template gets reviewed fast. A PR that just says "changes" gets sent back with questions.

---

## What we look for in a PR (review checklist)

This is what a reviewer (mentor or peer) checks before approving. Authors: check your own PR against this list *before* requesting review - it saves a round trip.

**Does it work?**
- [ ] CI (the **Build** check) is green. If it is red, the code does not compile or a test failed - not mergeable, full stop.
- [ ] The author described how they tested it (sim, real robot, or a clear reason it can't be tested yet). "It builds" alone is not "it works."

**Is it understandable?**
- [ ] The change is focused - one feature or fix, not a grab-bag.
- [ ] Commit messages and the PR description explain the *why*, not just the *what*.
- [ ] Code matches the style around it (naming, comments). New constants have a unit and a reason.

**Is it safe?**
- [ ] No generated files were hand-edited (`TunerConstants.java`).
- [ ] No secrets, no huge unrelated file dumps, no commented-out experiments left lying around without a note.
- [ ] Current limits, PID gains, and soft limits look sane - this is robot hardware; a bad number can break a mechanism.
- [ ] Docs were updated if behavior or constants changed.

**Is it current?**
- [ ] The branch is up to date with `main` (GitHub will warn if it is far behind).

Reviewers: leave specific comments, approve when it meets the bar, or request changes with a clear reason. Be kind - we are all learning. Authors: respond to every comment, push fixes to the same branch (the PR updates automatically), and re-request review.

---

## How a PR gets merged

1. CI passes (green check).
2. At least one mentor approves.
3. We use **Squash and merge** by default. This combines all the little commits on the branch into one clean commit on `main`, so the history stays readable. Make sure the squash commit message is meaningful (edit it on the merge screen if needed).
4. Delete the branch after merging.

Only mentors merge into `main`. That is not about trust - it is so there is always a second set of eyes, and so nobody accidentally merges something half-finished.

---

## Resolving conflicts without losing code

A merge conflict is **not** an error or a disaster. It just means two people changed the same lines and Git wants a human to decide. The dangerous part is panicking and throwing away one side. Don't.

When you merge `main` into your branch and there is a conflict, Git marks the spot like this:

```java
<<<<<<< HEAD
double kP = 0.5;   // your version
=======
double kP = 0.8;   // the version from main
>>>>>>> main
```

To resolve it **without losing anyone's work**:

1. Open each conflicted file (VS Code highlights them and gives "Accept Current / Accept Incoming / Accept Both" buttons).
2. **Read both sides and understand them.** The goal is the *correct* final code, which might be your side, their side, or a combination. Do not blindly "accept yours" - that is how the other person's work vanishes.
3. Delete the `<<<<<<<`, `=======`, and `>>>>>>>` marker lines, leaving the final correct code.
4. Build to confirm it compiles: `./gradlew build`.
5. Stage and commit the resolution:

```bash
git add .
git commit
```

If you ever feel lost in the middle of a conflict, you can bail out and start over - nothing is committed yet:

```bash
git merge --abort
```

Then get a mentor before trying again. **Asking for help here is the smart move, not the weak one.**

---

## How CI works

CI ("Continuous Integration") is the robot in [`.github/workflows/build.yml`](../.github/workflows/build.yml) that automatically compiles and tests our code every time you open or update a PR.

- It runs `./gradlew build` on a clean machine. That compiles everything and runs the JUnit tests.
- **Green check = it built and passed.** Safe to review/merge.
- **Red X = it is broken.** Click the check, then **Details**, to read the log and find the error. Fix it on your branch and push again - CI re-runs automatically.

CI is your safety net. It catches "works on my laptop but not anywhere else" before that code ever reaches `main`. A red PR never gets merged, so a broken build can't sneak onto the competition robot.

> "CD" (Continuous Deployment) means automatically shipping code to its target. We do **not** auto-deploy to the robot - deploying to a RoboRIO is a deliberate, in-person action (`./gradlew deploy`) done when the robot is safe to enable. Our pipeline stops at "continuously verified," which is exactly what we want for hardware.

---

## When something goes wrong: recovering work

Git almost never truly deletes committed work. If you think you lost something:

- **You committed it at some point?** `git reflog` shows everywhere your branch has been, even after a bad reset. Find the good state and `git checkout <that-id>` or `git reset --hard <that-id>` (careful with the second one).
- **Your branch got messed up but it is pushed to GitHub?** The version on GitHub is safe - you can re-clone or reset to `origin/your-branch`.
- **You are not sure?** Stop touching it and get a mentor *before* running more commands. The worst outcomes come from "fixing" a scary message with a destructive command you found online.

This is the whole reason we push branches often and protect `main`: there is always a safe copy somewhere.

---

## Branch protection (mentor setup)

So that "nobody pushes to `main`" is actually enforced and not just a promise, a mentor sets this up once on GitHub: **Settings → Branches → Add branch ruleset** for `main`, with:

- **Require a pull request before merging** (and require at least 1 approval).
- **Require status checks to pass** - select the **Build** check from our CI.
- **Require branches to be up to date before merging.**
- Block force-pushes and deletion of `main`.

With this on, the rules in this doc are guaranteed by GitHub itself, not left to memory.

---

## Quick command cheatsheet

```bash
# Start a new task
git checkout main
git pull
git checkout -b feat/my-thing

# Work, then save snapshots
git add .
git commit -m "Clear message about what changed"

# Back up to GitHub
git push -u origin feat/my-thing      # first push of a new branch
git push                              # every push after that

# Catch up with everyone else's merged work
git checkout main
git pull
git checkout feat/my-thing
git merge main

# Check where you are
git status
git branch
git log --oneline -10

# Emergency: undo an in-progress merge (nothing committed yet)
git merge --abort
```

---

## Glossary

- **Repository (repo):** the project and its full history.
- **Branch:** an independent line of work. `main` is one; your `feat/...` is another.
- **Commit:** a saved snapshot with a message.
- **Push / Pull:** upload your commits to GitHub / download others' commits from GitHub.
- **Pull Request (PR):** a request to merge your branch into `main`, with review and CI.
- **Merge conflict:** two changes to the same lines that Git needs a human to reconcile.
- **CI:** automation that builds and tests every PR.
- **Squash merge:** combine a branch's commits into one tidy commit on `main`.

---

Team 8041 - Lake City Ultrabots, 2026
