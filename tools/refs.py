"""Reference registry. Every entry carries how it was verified and on which date.

Verification method used in this build: web search of publisher, index, library or author-hosted
records (Crossref's API was not reachable from the build environment). A reference marked
"verified" had its authors, year, title and venue (journal, volume, pages or ISBN) confirmed on at
least one such record. A reference marked "partial" lists what was NOT confirmed. Nothing here was
checked by reading the full text unless the note says so.
"""

from __future__ import annotations

CHECKED = "2026-11-25"
METHOD = "web search of publisher/index/library record"


def _v(url, note="", status="verified"):
    return {"status": status, "method": METHOD, "checked": CHECKED, "evidence_url": url,
            "note": note}


REFS = {
    "zn1942": dict(
        kind="paper", authors="Ziegler, J. G.; Nichols, N. B.", year=1942,
        title="Optimum settings for automatic controllers",
        venue="Transactions of the ASME", volume="64", pages="759-768", doi=None,
        role="Origin of the open-loop (reaction curve) and closed-loop (ultimate gain) PID rules.",
        limits="Tuned for a pneumatic controller and quarter-amplitude decay; the loops are "
               "often oscillatory by modern criteria.",
        verification=_v("https://elec3004.uqcloud.net/2015/assignments/PS4-Z_N_Original.pdf",
                        "Facsimile of the paper and several citation lists agree on volume and "
                        "pages.")),
    "cc1953": dict(
        kind="paper", authors="Cohen, G. H.; Coon, G. A.", year=1953,
        title="Theoretical consideration of retarded control",
        venue="Transactions of the ASME", volume="75", pages="827-834", doi=None,
        role="Reaction-curve tuning rules that account for large dead time.",
        limits="Rules are for quarter-amplitude decay; the title is cited with small wording "
               "variants in the literature.",
        verification=_v("https://informahealthcare.com/doi/ref/10.3109/08958379509029718",
                        "Authors, year, volume and pages agree across several third-party "
                        "reference lists; the primary page was not seen. Title wording varies.",
                        status="partial")),
    "tl1992": dict(
        kind="paper", authors="Tyreus, B. D.; Luyben, W. L.", year=1992,
        title="Tuning PI controllers for integrator/dead time processes",
        venue="Industrial & Engineering Chemistry Research", volume=None, pages=None,
        doi="10.1021/ie00011a029",
        role="Source of the Tyreus-Luyben ultimate-point rules (less oscillatory than Z-N).",
        limits="Developed for PI on integrator plus dead time processes; use for PID is an "
               "extension in common practice.",
        verification=_v("https://www.bohrium.com/paper-details/tuning-pi-controllers-for-"
                        "integrator-dead-time-processes/812294451635421184-105711",
                        "Title, authors, year and DOI seen on an index record. Volume and pages "
                        "were not confirmed and are left blank.", status="partial")),
    "rmm1986": dict(
        kind="paper", authors="Rivera, D. E.; Morari, M.; Skogestad, S.", year=1986,
        title="Internal model control: PID controller design",
        venue="Industrial & Engineering Chemistry Process Design and Development",
        volume="25", issue="1", pages="252-265", doi="10.1021/i200032a041",
        role="Derives PID settings from the IMC procedure with one tuning parameter.",
        limits="Model based; assumes a reasonably accurate low-order model.",
        verification=_v("https://skoge.folk.ntnu.no/publications/1986/Rivera86/README.html",
                        "Author-hosted record gives journal, volume, issue and pages.")),
    "skogestad2003": dict(
        kind="paper", authors="Skogestad, S.", year=2003,
        title="Simple analytic rules for model reduction and PID controller tuning",
        venue="Journal of Process Control", volume="13", issue="4", pages="291-309",
        doi="10.1016/S0959-1524(02)00062-8",
        role="SIMC rules and the half rule for effective delay.",
        limits="An erratum exists (J. Process Control 14, 2004, p. 465).",
        verification=_v("https://skoge.folk.ntnu.no/publications/2003/tuningPID/",
                        "Author-hosted record and an index record agree; the erratum was seen.")),
    "ah1984": dict(
        kind="paper", authors="Astrom, K. J.; Hagglund, T.", year=1984,
        title="Automatic tuning of simple regulators with specifications on phase and "
              "amplitude margins",
        venue="Automatica", volume="20", issue="5", pages="645-651",
        doi="10.1016/0005-1098(84)90014-1",
        role="Relay feedback experiment for estimating the ultimate gain and period.",
        limits="The describing-function estimate is approximate for non-sinusoidal limit cycles.",
        verification=_v("https://lup.lub.lu.se/record/8496536",
                        "University repository record with abstract, volume, issue, pages.")),
    "ah1995": dict(
        kind="book", authors="Astrom, K. J.; Hagglund, T.", year=1995,
        title="PID Controllers: Theory, Design, and Tuning, 2nd ed.",
        venue="ISA (Instrument Society of America)", isbn="1-55617-516-7",
        role="Standard reference on PID structure, anti-windup, filtering and tuning.",
        limits="Pre-dates much current autotuning practice.",
        verification=_v("https://lup.lub.lu.se/record/1aa6ddcb-00bd-4d86-92d2-b66d981c534f",
                        "University repository record gives publisher, year, ISBN, 343 pages.")),
    "harel1987": dict(
        kind="paper", authors="Harel, D.", year=1987,
        title="Statecharts: a visual formalism for complex systems",
        venue="Science of Computer Programming", volume="8", issue="3", pages="231-274",
        doi="10.1016/0167-6423(87)90035-9",
        role="Hierarchy, concurrency and broadcast extensions to flat state machines.",
        limits="Specification formalism; flat machines in this repository do not use these extensions.",
        verification=_v("https://scilit.com/publications/94e490c2a64c2133e4ece3da7c502649",
                        "Index record and a facsimile of the first page agree on journal, volume, pages and DOI.")),
    "moore1956": dict(
        kind="chapter", authors="Moore, E. F.", year=1956,
        title="Gedanken-experiments on sequential machines",
        venue="Automata Studies (Annals of Mathematics Studies 34), ed. C. E. Shannon and J. McCarthy, Princeton University Press",
        pages="129-153",
        role="Defines the machine whose output depends on the state only (Moore machine).",
        limits="Theory of state identification experiments, not a control implementation guide.",
        verification=_v("https://people.irisa.fr/Nicolas.Markey/biblio/am34-Moo",
                        "Bibliographic record and a facsimile of the chapter agree on title, editors, series and pages.")),
    "zadeh1965": dict(
        kind="paper", authors="Zadeh, L. A.", year=1965, title="Fuzzy sets",
        venue="Information and Control", volume="8", issue="3", pages="338-353",
        doi="10.1016/S0019-9958(65)90241-X",
        role="Defines fuzzy sets and membership functions.",
        limits="Foundational mathematics, not a control design method.",
        verification=_v("https://www.mathnet.ru/eng/fssc12",
                        "Index record gives the English version volume, issue, pages and DOI.")),
    "mamdani1975": dict(
        kind="paper", authors="Mamdani, E. H.; Assilian, S.", year=1975,
        title="An experiment in linguistic synthesis with a fuzzy logic controller",
        venue="International Journal of Man-Machine Studies", volume="7", issue="1",
        pages="1-13", doi="10.1016/S0020-7373(75)80002-2",
        role="First fuzzy logic controller applied to a (laboratory) steam engine plant.",
        limits="Laboratory plant; no stability analysis.",
        verification=_v("https://scilit.com/publications/106170e3e1e3ddf19fc8b5ba1fd30628",
                        "Index record gives venue, volume, issue, pages and DOI.")),
    "mayne2000": dict(
        kind="paper", authors="Mayne, D. Q.; Rawlings, J. B.; Rao, C. V.; Scokaert, P. O. M.",
        year=2000, title="Constrained model predictive control: Stability and optimality",
        venue="Automatica", volume="36", issue="6", pages="789-814",
        doi="10.1016/S0005-1098(99)00214-9",
        role="Survey of the conditions (terminal cost/constraint) that give MPC stability.",
        limits="The controller built in this repository has no terminal ingredients, so these "
               "guarantees do not apply to it.",
        verification=_v("https://dblp.uni-trier.de/rec/journals/automatica/MayneRRS00.html",
                        "dblp and an index record agree on venue, volume, issue and pages.")),
    "rmd2017": dict(
        kind="book", authors="Rawlings, J. B.; Mayne, D. Q.; Diehl, M. M.", year=2017,
        title="Model Predictive Control: Theory, Computation, and Design, 2nd ed.",
        venue="Nob Hill Publishing", isbn="978-0-9759377-3-0",
        role="Textbook treatment of linear and nonlinear MPC, estimation and stability.",
        limits="Theory heavy; implementation details of real-time solvers are limited.",
        verification=_v("https://engineering.ucsb.edu/~jbraw/mpc/MPC-book-2nd-edition-1st-"
                        "printing.pdf", "Author-hosted front matter gives publisher and ISBN.")),
    "pillay1989": dict(
        kind="paper", authors="Pillay, P.; Krishnan, R.", year=1989,
        title="Modeling, simulation, and analysis of permanent-magnet motor drives, Part II: "
              "The brushless DC motor drive",
        venue="IEEE Transactions on Industry Applications", volume="25", issue="2",
        pages="274-279", doi="10.1109/28.25542",
        role="Phase-variable BLDC model with trapezoidal back-EMF used for drive simulation.",
        limits="Abstract-level verification only; the equations used here were written "
               "independently from the standard model.",
        verification=_v("https://scilit.com/publications/5d5f2b6c3013f1d04f02d2ecb398b108",
                        "Index record gives venue, volume, issue, pages and DOI.")),
    "vdb1988": dict(
        kind="paper", authors="van der Broeck, H. W.; Skudelny, H.-C.; Stanke, G. V.",
        year=1988,
        title="Analysis and realization of a pulsewidth modulator based on voltage space vectors",
        venue="IEEE Transactions on Industry Applications", volume="24", issue="1",
        pages="142-150", doi="10.1109/28.87265",
        role="Space-vector PWM and its relation to carrier-based modulation with common-mode "
             "injection.",
        limits="The min-max injection used here is the carrier-based equivalent, not a "
               "line-by-line reproduction.",
        verification=_v("https://www.scilit.net/publications/eea5ffd1eee74a45a6c60791bd4edd41",
                        "Index record gives venue, volume, issue, pages and DOI.")),
    "park1929": dict(
        kind="paper", authors="Park, R. H.", year=1929,
        title="Two-reaction theory of synchronous machines: generalized method of analysis, "
              "Part I",
        venue="Transactions of the AIEE", volume="48", issue="3", pages="716-727",
        doi="10.1109/T-AIEE.1929.5055275",
        role="Origin of the d-q reference-frame transformation.",
        limits="Historical; modern drives use the amplitude-invariant form defined by convention.",
        verification=_v("https://colab.ws/articles/10.1109%2FT-AIEE.1929.5055275",
                        "Index records agree on volume, issue and pages.")),
    "krause2013": dict(
        kind="book", authors="Krause, P. C.; Wasynczuk, O.; Sudhoff, S. D.; Pekarek, S. D.",
        year=2013, title="Analysis of Electric Machinery and Drive Systems, 3rd ed.",
        venue="Wiley-IEEE Press", isbn="978-1-118-02429-4",
        role="Reference-frame theory, PMSM and BLDC models, averaged converter models.",
        limits="Textbook; parameters for a specific machine must come from its datasheet.",
        verification=_v("https://engineering.purdue.edu/ECE/Research/Areas/PES/Books/AEMDS",
                        "Publisher-affiliated page and a retailer record agree on edition, "
                        "year and ISBN.")),
    "am2021": dict(
        kind="book", authors="Astrom, K. J.; Murray, R. M.", year=2021,
        title="Feedback Systems: An Introduction for Scientists and Engineers, 2nd ed.",
        venue="Princeton University Press", isbn="978-0-691-19398-4",
        role="Modern introduction to feedback, modeling, stability, PID and robustness.",
        limits="Broad textbook; industrial specifics are limited.",
        verification=_v("https://vufind.lboro.ac.uk/Record/568893/Description",
                        "Library record gives publisher, year and ISBN.")),
    "seborg2016": dict(
        kind="book", authors="Seborg, D. E.; Edgar, T. F.; Mellichamp, D. A.; Doyle, F. J.",
        year=2016, title="Process Dynamics and Control, 4th ed.", venue="Wiley",
        isbn="978-1-119-28591-5",
        role="Process-control textbook: FOPDT models, reaction-curve methods, tuning rules.",
        limits="Which tuning rules and identification methods it covers was not checked "
               "page by page in this build.",
        verification=_v("https://www.mathworks.com/academia/books/process-dynamics-and-control-"
                        "seborg.html", "Publisher listing gives edition, year and ISBN.")),
}

STANDARDS = {
    "iec61131_3": dict(
        code="IEC 61131-3:2013", title="Programmable controllers - Part 3: Programming languages",
        edition="3.0", published="2013-02-20", body="IEC",
        relevance="Defines function blocks TON, TOF, CTU, R_TRIG, F_TRIG, SR and the scan "
                  "execution model the PLC emulation follows.",
        status_note="The IEC webstore lists a more recent edition, IEC 61131-3:2025. The "
                    "2013 edition is the one verified here and may be withdrawn.",
        verification=_v("https://webstore.iec.ch/publication/4552",
                        "IEC webstore record; the full text was not read (paid standard).")),
    "iso5048": dict(
        code="ISO 5048:1989", title="Continuous mechanical handling equipment - Belt conveyors "
        "with carrying idlers - Calculation of operating power and tensile forces",
        edition="2", published="1989-09", body="ISO",
        relevance="Scope: resistances to motion, driving force and power on the driving pulley. "
                  "Context for the lumped resistance terms of the conveyor model; none of its "
                  "formulas or coefficients is used.",
        status_note="ISO's page lists the edition as current; the English page says it was last "
                    "reviewed and confirmed in 2019, the Spanish page says 2026 (unreconciled).",
        verification=_v("https://www.iso.org/standard/11069.html",
                        "ISO catalogue record and abstract; the full text is paid and was not read.")),
}

# Candidates named in the project brief but NOT verified in this build.
UNVERIFIED_CANDIDATES = [
    ("Smith two-point FOPDT identification (28.3 % / 63.2 %)",
     "Used in notebook 02. Textbooks present it, but the primary source was not located. "
     "Treat the method as standard practice, not as a cited result."),
    ("Blaschke 1972, field orientation (Siemens Review 34)",
     "Not searched in this build."),
    ("IEC 61508 (functional safety), IEC 61800 (adjustable speed drives), ISA-5.1, "
     "ISO 13849-1", "Named as relevant in the brief; edition and scope not verified."),
    ("Recent (2020 onward) papers on fuzzy PID, MPC for drives and sensorless FOC",
     "No recent literature was searched, so the research registers have no post-2021 papers."),
]


def cite(key: str) -> str:
    r = REFS[key]
    bits = [f"{r['authors']} ({r['year']}). {r['title']}."]
    if r.get("venue"):
        v = r["venue"]
        if r.get("volume"):
            v += f", {r['volume']}"
            if r.get("issue"):
                v += f"({r['issue']})"
        if r.get("pages"):
            v += f", {r['pages']}"
        bits.append(v + ".")
    if r.get("isbn"):
        bits.append(f"ISBN {r['isbn']}.")
    if r.get("doi"):
        bits.append(f"doi:{r['doi']}.")
    return " ".join(bits)


def status_tag(key: str) -> str:
    return REFS[key]["verification"]["status"]
