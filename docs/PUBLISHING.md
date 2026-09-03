# Before flipping this repo public — the checklist

This repo is PRIVATE because it contains material we can't publish as-is.
Do every item below before making it public:

1. **TTC-derived tire values** (`car_data.py`, tires block; also the
   parameter sheet + BREAKDOWN tire tables): the 9 fitted coefficients are
   derived from TTC Round 9 data, restricted by the TTC agreement the team
   signed. Before going public, either get explicit clearance on the TTC
   forum, or replace the values with labeled representative placeholders
   and keep the real fit in a private overlay. Ask Milliken if unsure —
   losing TTC access costs the team real money.
2. **`ttc/` folder** — already gitignored; verify nothing from it ever got
   committed (`git log --stat | grep -i ttc`).
3. **Third-party PDFs** — remove `AMK Racing Kit Datasheet.pdf` (AMK's
   copyright; link to AMK's download page instead) and
   `1ME295B Project Report_FINALJW-2.pdf` (another student's report — get
   their permission or remove and cite).
4. **Pick a license** — none yet, so the repo is legally all-rights-
   reserved. MIT recommended for tool repos.
5. Skim `car_data.py` and the run notes for anything team-sensitive
   (driver names/weights are in there: 156 lb driver).
