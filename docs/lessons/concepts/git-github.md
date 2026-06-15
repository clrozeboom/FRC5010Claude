# Deep Dive: Git & GitHub

> Background for the [intake + dual-flywheel lesson](../intake-flywheel-lesson.md). Read this if
> *branch*, *commit*, *push*, or *pull request* are new.

**Git** is a tool that takes snapshots of your project so you can save progress, undo mistakes,
and combine work with others. **GitHub** is a website that stores those snapshots online and adds
collaboration features (pull requests, reviews, issues). Git runs on your computer; GitHub is the
shared home for the project.

---

## 1. The pieces

- **Repository ("repo")** — your project folder, with a hidden `.git` history attached.
- **Fork** — your own copy of someone else's repo, living under your GitHub account.
- **Clone** — a copy of a repo on your computer.
- **Commit** — one saved snapshot, with a short message describing the change.
- **Branch** — a separate line of work, so experiments don't disturb the main code.
- **Remote / `origin`** — the online copy your local repo syncs with (usually called `origin`).
- **Pull request ("PR")** — a request to merge one branch's changes into another, with a place to
  review and discuss them.

---

## 2. Why commit often

Each commit is a **save point**. If a later change breaks something, you can look at — or return
to — any earlier commit. Small, frequent commits with clear messages make it easy to see what
changed and when. That's why the lesson sprinkles **💾 Checkpoint — commit** prompts: each one is
a safe spot to come back to.

A good commit message says *what* changed, briefly:

```
Add IntakeArm subsystem (LQR)
```

---

## 3. The three areas (working dir → staging → commit)

When you edit files, Git sees three places:

1. **Working directory** — your actual files, as you've edited them.
2. **Staging area** — changes you've marked to include in the next commit (`git add`).
3. **History** — the committed snapshots (`git commit`).

```bash
git status          # what changed, and what's staged
git add .           # stage ALL current changes (the "." means "everything here")
git add Shooter.java # or stage just one file
git commit -m "Add Shooter flywheel"   # snapshot the staged changes
```

Think of `add` as "put these on the tray" and `commit` as "save the tray as one snapshot."

---

## 4. Branches

A **branch** is an independent line of commits. You make one so your lesson work stays separate
from the project's main line until it's ready.

```bash
git switch -c my-intake-lesson   # create a new branch AND move onto it
git switch main                  # hop back to the main branch
git switch my-intake-lesson      # hop back to your work
git branch                       # list branches; the current one has a *
```

Switching branches changes the files in your folder to match that branch — so commit (or stash)
before you switch.

---

## 5. Sharing your work (push) and getting others' (pull)

Your commits live only on your computer until you **push** them to GitHub:

```bash
git push -u origin my-intake-lesson
```

The `-u origin my-intake-lesson` part links your local branch to a branch of the same name on
`origin` (GitHub). After the first time, plain `git push` is enough.

To bring down changes others pushed:

```bash
git pull origin main
```

---

## 6. The everyday loop

This is the rhythm you'll repeat at every checkpoint:

```bash
# ...make some edits...
git status                          # see what changed
git add .                           # stage it
git commit -m "what I just did"     # save a snapshot
git push                            # upload it
```

---

## 7. Pull requests

When a branch is ready, open a **pull request** to merge it into `main`. On GitHub, after you
push, you'll see a **Compare & pull request** button. A PR:

- shows the exact diff (lines added/removed),
- lets others review and comment,
- runs automated checks (the project's tests),
- becomes part of `main` when merged.

You don't merge your own learning PRs into the shared project unless a mentor asks — but opening
one is how you show your work and get feedback.

---

## 8. Undo, gently

- **Discard edits to a file** (before committing): `git restore Shooter.java`
- **Unstage** a file you `add`-ed but haven't committed: `git restore --staged Shooter.java`
- **Look at history:** `git log --oneline` (press `q` to quit the viewer)
- **Go look at an old commit** without changing your branch: `git switch --detach <commit-id>`,
  then `git switch my-intake-lesson` to come back.

When in doubt, **commit first** — a snapshot you can return to makes everything reversible.

---

## 9. Merge conflicts (just so they're not scary)

If two branches change the *same lines*, Git can't pick automatically and marks a **conflict** in
the file like this:

```
<<<<<<< HEAD
your version
=======
the other version
>>>>>>> other-branch
```

You edit the file to the version you want, delete the `<<<`/`===`/`>>>` markers, then
`git add` and `git commit`. For this lesson, working on your own branch, you'll rarely hit one.

---

## 10. `.gitignore`

Some files shouldn't be saved (build output, local settings). A file named `.gitignore` lists
patterns Git should skip. This project already has one — you usually don't touch it.

---

### Back to the lesson

- [Module 0 — Get set up (and meet Git/GitHub)](../intake-flywheel-lesson.md#module-0--get-set-up-and-meet-gitgithub)
- The project's full setup guide: [docs/student-setup.md](../../student-setup.md)
