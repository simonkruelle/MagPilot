# Weekly lab updates

Short project updates, organised by week. Review the Markdown version directly
on GitHub or open the two-page PDF for the illustrated slides.

| Work week | Meeting | Focus | Review | Slides |
|---|---|---|---|---|
| **Week 2 · 5–11 Oct** | **12 October 2026** | Teleoperation collection; board repair/diagnostics; magnet comparison; pen printing. In progress, as of 6 Oct. | [Read update](MagPilot_Lab_Update_2026-10-12.md) | [PDF](MagPilot_Lab_Update_2026-10-12.pdf) · [Word](MagPilot_Lab_Update_2026-10-12.docx) |
| **Week 1 · 28 Sep–4 Oct** | 5 October 2026 | Character protocol, named participants and operator-controlled recording. | [Read update](MagPilot_Lab_Update_2026-10-05.md) | [PDF](MagPilot_Lab_Update_2026-10-05.pdf) · [Word](MagPilot_Lab_Update_2026-10-05.docx) |

Week 1 began **28 September 2026**; weeks run Monday–Sunday. Monday meetings
review the previous work week: **12 October reviews Week 2**.
Reported checks, printing in progress and planned experiments are identified separately.

Rebuild the reports from the repository root:

```bash
python3 tools/build_lab_report_pdf.py --refresh-timeline-asset
python3 tools/build_lab_report.py
python3 tools/build_week2_report.py
```

Use a Python environment with Matplotlib and Pillow for the PDF/Week 2 build.
The standalone Week 1 Word generator uses the standard library.
