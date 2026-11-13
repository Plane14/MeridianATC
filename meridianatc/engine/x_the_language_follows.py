from __future__ import annotations

from ._shared import (
    LANGUAGE_NAMES,
    languages_for,
    profile_for,
)


class _The_language_follows:
    # ------------------------------------------------------------------
    # the language follows the aeroplane
    # ------------------------------------------------------------------

    def follow_languages(self, ident: str, announce: bool = False) -> list[str]:
        """Listen for whatever is worked where the aeroplane now is.

        English is always in the set. The local language joins it over a state
        that works one this system can speak, and drops out again on the way
        out. Nothing about this is configurable, because nothing about it is a
        preference: it is where you are.
        """
        worked = languages_for(ident)
        if self.pinned_languages:
            worked = [l for l in worked if l in self.pinned_languages] or ["en"]
        if worked == self.languages:
            return worked

        self.languages = worked
        self.recognizer.set_languages(worked)
        if self.last_language not in worked:
            self.last_language = worked[0]

        if announce or len(worked) > 1:
            local = [l for l in worked if l != "en"]
            if local:
                name = LANGUAGE_NAMES.get(local[0], local[0])
                self.emit("info",
                          f"{profile_for(ident).name}: {name} is worked here "
                          f"alongside English. Speak either; the controller "
                          f"answers in the one you used.",
                          key="log.bilingual", where=profile_for(ident).name,
                          language=local[0])
            elif announce:
                self.emit("info", f"{profile_for(ident).name}: English on frequency.",
                          key="log.english_only", where=profile_for(ident).name)
        return worked

