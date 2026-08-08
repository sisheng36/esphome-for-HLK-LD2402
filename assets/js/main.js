/* HLK-LD2402 介绍页交互脚本 */
(function () {
  "use strict";

  /* ---------- 滚动显现动画 ---------- */
  const revealEls = document.querySelectorAll(".reveal");
  if ("IntersectionObserver" in window) {
    const io = new IntersectionObserver(
      (entries) => {
        entries.forEach((entry) => {
          if (entry.isIntersecting) {
            entry.target.classList.add("visible");
            io.unobserve(entry.target);
          }
        });
      },
      { threshold: 0.12 }
    );
    revealEls.forEach((el) => io.observe(el));
  } else {
    revealEls.forEach((el) => el.classList.add("visible"));
  }

  /* ---------- 数字滚动计数 ---------- */
  const counters = document.querySelectorAll("[data-count]");
  if ("IntersectionObserver" in window && counters.length) {
    const cio = new IntersectionObserver(
      (entries) => {
        entries.forEach((entry) => {
          if (!entry.isIntersecting) return;
          const el = entry.target;
          cio.unobserve(el);
          const target = parseInt(el.dataset.count, 10);
          const dur = 1100;
          const t0 = performance.now();
          const tick = (t) => {
            const p = Math.min((t - t0) / dur, 1);
            el.textContent = Math.round(target * (1 - Math.pow(1 - p, 3)));
            if (p < 1) requestAnimationFrame(tick);
          };
          requestAnimationFrame(tick);
        });
      },
      { threshold: 0.6 }
    );
    counters.forEach((el) => cio.observe(el));
  }

  /* ---------- 缩略图点击放大（轻量灯箱） ---------- */
  const thumbs = document.querySelectorAll(".img-thumbs img");
  if (thumbs.length) {
    const overlay = document.createElement("div");
    overlay.className = "lightbox";
    overlay.innerHTML =
      '<img alt="HLK-LD2402 大图"><button class="lightbox-close" aria-label="关闭">✕</button>';
    document.body.appendChild(overlay);

    const boxImg = overlay.querySelector("img");
    const open = (src) => {
      boxImg.src = src;
      overlay.classList.add("open");
    };
    const close = () => overlay.classList.remove("open");

    thumbs.forEach((img) =>
      img.addEventListener("click", () => open(img.src))
    );
    overlay.addEventListener("click", (e) => {
      if (e.target === overlay || e.target.classList.contains("lightbox-close")) close();
    });
    document.addEventListener("keydown", (e) => {
      if (e.key === "Escape") close();
    });
  }

  /* ---------- 导航滚动状态 ---------- */
  const nav = document.querySelector(".nav");
  window.addEventListener("scroll", () => {
    nav.classList.toggle("nav-scrolled", window.scrollY > 40);
  }, { passive: true });
})();
